/*
 * Copyright (c) 2017 Linaro Limited
 * Copyright (c) 2018-2019 Foundries.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Uses some original concepts by:
 *         Joakim Eriksson <joakime@sics.se>
 *         Niclas Finne <nfi@sics.se>
 *         Joel Hoglund <joel@sics.se>
 */

/**************************************************************************************************/
/* Includes                                                                                       */
/**************************************************************************************************/
#define LOG_MODULE_NAME net_lwm2m_udp
#define LOG_LEVEL	CONFIG_LWM2M_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <ctype.h>
#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/types.h>

#include <fcntl.h>
#include <init.h>
#include <net/http_parser_url.h>
#include <net/net_ip.h>
#include <net/socket.h>
#include <sys/printk.h>

#if defined(CONFIG_LWM2M_DTLS_SUPPORT)
#include <mbedtls/ssl.h>
#include <net/tls_credentials.h>

#endif
#if defined(CONFIG_DNS_RESOLVER)
#include <net/dns_resolve.h>
#endif

#include "lwm2m_engine.h"
#include "lwm2m_object.h"
#include "lwm2m_transport.h"
#include "lwm2m_util.h"
#include "lwm2m_message_handling.h"

/**************************************************************************************************/
/* Local Function Prototypes                                                                      */
/**************************************************************************************************/
#if defined(CONFIG_LWM2M_DTLS_SUPPORT) && defined(CONFIG_TLS_CREDENTIALS)
static int load_tls_credential(struct lwm2m_ctx *client_ctx, uint16_t res_id,
			       enum tls_credential_type type);
#endif
#if defined(CONFIG_LWM2M_DTLS_SUPPORT)
static void filter_cipher_list(int fd);
#endif

static int lwm2m_transport_udp_setup(struct lwm2m_ctx *client_ctx, char *url, bool is_firmware_uri);
static int lwm2m_transport_udp_open(struct lwm2m_ctx *client_ctx);
static int lwm2m_transport_udp_start(struct lwm2m_ctx *client_ctx);
static int lwm2m_transport_udp_suspend(struct lwm2m_ctx *client_ctx, bool should_close);
static int lwm2m_transport_udp_resume(struct lwm2m_ctx *client_ctx);
static int lwm2m_transport_udp_close(struct lwm2m_ctx *client_ctx);
static int lwm2m_transport_udp_send(struct lwm2m_ctx *client_ctx, const uint8_t *data,
				    uint32_t len);
static int lwm2m_transport_udp_recv(struct lwm2m_ctx *client_ctx);
static int lwm2m_transport_udp_is_connected(struct lwm2m_ctx *client_ctx);
static void lwm2m_transport_udp_tx_pending(struct lwm2m_ctx *client_ctx, bool pending);
static char *lwm2m_transport_udp_print_addr(struct lwm2m_ctx *client_ctx,
					    const struct sockaddr *addr);

/**************************************************************************************************/
/* Local Data Definitions                                                                         */
/**************************************************************************************************/
static struct lwm2m_transport_procedure udp_transport = {
	.setup = lwm2m_transport_udp_setup,
	.open = lwm2m_transport_udp_open,
	.start = lwm2m_transport_udp_start,
	.suspend = lwm2m_transport_udp_suspend,
	.resume = lwm2m_transport_udp_resume,
	.close = lwm2m_transport_udp_close,
	.send = lwm2m_transport_udp_send,
	.recv = lwm2m_transport_udp_recv,
	.is_connected = lwm2m_transport_udp_is_connected,
	.tx_pending = lwm2m_transport_udp_tx_pending,
	.print_addr = lwm2m_transport_udp_print_addr,
};

/**************************************************************************************************/
/* Global Function Definitions                                                                    */
/**************************************************************************************************/
int lwm2m_transport_udp_register(void)
{
	return lwm2m_transport_register("udp", &udp_transport);
}

/**************************************************************************************************/
/* Local Function Definitions                                                                     */
/**************************************************************************************************/
#if defined(CONFIG_LWM2M_DTLS_SUPPORT) && defined(CONFIG_TLS_CREDENTIALS)
static int load_tls_credential(struct lwm2m_ctx *client_ctx, uint16_t res_id,
			       enum tls_credential_type type)
{
	int ret = 0;
	void *cred = NULL;
	uint16_t cred_len;
	uint8_t cred_flags;
	char pathstr[MAX_RESOURCE_LEN];

	/* ignore error value */
	tls_credential_delete(client_ctx->tls_tag, type);

	snprintk(pathstr, sizeof(pathstr), "0/%d/%u", client_ctx->sec_obj_inst, res_id);

	ret = lwm2m_engine_get_res_buf(pathstr, &cred, NULL, &cred_len, &cred_flags);
	if (ret < 0) {
		LOG_ERR("Unable to get resource data for '%s'", pathstr);
		return ret;
	}

	if (cred_len == 0) {
		LOG_ERR("Credential data is empty");
		return -EINVAL;
	}

	ret = tls_credential_add(client_ctx->tls_tag, type, cred, cred_len);
	if (ret < 0) {
		LOG_ERR("Error setting cred tag %d type %d: Error %d", client_ctx->tls_tag, type,
			ret);
	}

	return ret;
}
#endif /* CONFIG_LWM2M_DTLS_SUPPORT && CONFIG_TLS_CREDENTIALS*/

#if defined(CONFIG_LWM2M_DTLS_SUPPORT)
static void filter_cipher_list(int fd)
{
	int input_cipher_list[CONFIG_LWM2M_TLS_MAX_CIPHERSUITES];
	int output_cipher_list[CONFIG_NET_SOCKETS_TLS_MAX_CIPHERSUITES - 1];
	uint32_t in_list_len = sizeof(input_cipher_list);
	uint32_t out_list_len = 0;
	const struct mbedtls_ssl_ciphersuite_t *cs;
	int ret;
	int i;

	/* Fetch the current list of ciphers */
	memset(input_cipher_list, 0, sizeof(input_cipher_list));
	ret = getsockopt(fd, SOL_TLS, TLS_CIPHERSUITE_LIST, input_cipher_list, &in_list_len);
	if (ret < 0) {
		LOG_ERR("Could not fetch cipher list (%d). Not filtering.", errno);
	} else {
		if (in_list_len == sizeof(input_cipher_list)) {
			LOG_WRN("Input cipher list is max length, possibly truncated");
		}

		/* Copy PSK ciphers into the output list */
		for (i = 0; (i < (in_list_len / sizeof(int))) &&
			    (out_list_len < (CONFIG_NET_SOCKETS_TLS_MAX_CIPHERSUITES - 1));
		     i++) {
			cs = mbedtls_ssl_ciphersuite_from_id(input_cipher_list[i]);
			if (mbedtls_ssl_ciphersuite_uses_psk(cs)) {
				output_cipher_list[out_list_len++] = input_cipher_list[i];
			}
		}

		if (out_list_len >= (CONFIG_NET_SOCKETS_TLS_MAX_CIPHERSUITES - 1)) {
			LOG_WRN("Output cipher list is max length, possibly truncated");
		}

		/* Set the new cipher list */
		ret = setsockopt(fd, SOL_TLS, TLS_CIPHERSUITE_LIST, output_cipher_list,
				 (out_list_len * sizeof(int)));
		if (ret < 0) {
			LOG_ERR("Could not set filtered list: %d", errno);
		}
	}
}
#endif /* CONFIG_LWM2M_DTLS_SUPPORT */

static int lwm2m_transport_udp_setup(struct lwm2m_ctx *client_ctx, char *url, bool is_firmware_uri)
{
	struct http_parser_url parser;
#if defined(CONFIG_LWM2M_DNS_SUPPORT)
	struct addrinfo *res, hints = {0};
#endif
	int ret;
	uint16_t off, len;
	uint8_t tmp;

	LOG_DBG("Parse url: %s", url);

	http_parser_url_init(&parser);
	ret = http_parser_parse_url(url, strlen(url), 0, &parser);
	if (ret < 0) {
		LOG_ERR("Invalid url: %s", url);
		return -ENOTSUP;
	}

	off = parser.field_data[UF_SCHEMA].off;
	len = parser.field_data[UF_SCHEMA].len;

	/* check for supported protocol */
	if (strncmp(url + off, "coaps", len) != 0) {
		return -EPROTONOSUPPORT;
	}

	/* check for DTLS requirement */
	client_ctx->use_dtls = false;
	if (len == 5U && strncmp(url + off, "coaps", len) == 0) {
#if defined(CONFIG_LWM2M_DTLS_SUPPORT)
		client_ctx->use_dtls = true;
#else
		return -EPROTONOSUPPORT;
#endif /* CONFIG_LWM2M_DTLS_SUPPORT */
	}

	if (!(parser.field_set & (1 << UF_PORT))) {
		if (is_firmware_uri && client_ctx->use_dtls) {
			/* Set to default coaps firmware update port */
			parser.port = CONFIG_LWM2M_FIRMWARE_PORT_SECURE;
		} else if (is_firmware_uri) {
			/* Set to default coap firmware update port */
			parser.port = CONFIG_LWM2M_FIRMWARE_PORT_NONSECURE;
		} else {
			/* Set to default LwM2M server port */
			parser.port = CONFIG_LWM2M_PEER_PORT;
		}
	}

	off = parser.field_data[UF_HOST].off;
	len = parser.field_data[UF_HOST].len;

#if defined(CONFIG_LWM2M_DTLS_SUPPORT)
	/** copy url pointer to be used in socket */
	client_ctx->desthostname = url + off;
	client_ctx->desthostnamelen = len;
#endif

	/* truncate host portion */
	tmp = url[off + len];
	url[off + len] = '\0';

	/* initialize remote_addr */
	(void)memset(&client_ctx->remote_addr, 0, sizeof(client_ctx->remote_addr));

	/* try and set IP address directly */
	client_ctx->remote_addr.sa_family = AF_INET6;
	ret = net_addr_pton(AF_INET6, url + off,
			    &((struct sockaddr_in6 *)&client_ctx->remote_addr)->sin6_addr);
	/* Try to parse again using AF_INET */
	if (ret < 0) {
		client_ctx->remote_addr.sa_family = AF_INET;
		ret = net_addr_pton(AF_INET, url + off,
				    &((struct sockaddr_in *)&client_ctx->remote_addr)->sin_addr);
	}

	if (ret < 0) {
#if defined(CONFIG_LWM2M_DNS_SUPPORT)
#if defined(CONFIG_NET_IPV6) && defined(CONFIG_NET_IPV4)
		hints.ai_family = AF_UNSPEC;
#elif defined(CONFIG_NET_IPV6)
		hints.ai_family = AF_INET6;
#elif defined(CONFIG_NET_IPV4)
		hints.ai_family = AF_INET;
#else
		hints.ai_family = AF_UNSPEC;
#endif /* defined(CONFIG_NET_IPV6) && defined(CONFIG_NET_IPV4) */
		hints.ai_socktype = SOCK_DGRAM;
		ret = getaddrinfo(url + off, NULL, &hints, &res);
		if (ret != 0) {
			LOG_ERR("Unable to resolve address");
			/* DNS error codes don't align with normal errors */
			ret = -ENOENT;
			goto cleanup;
		}

		memcpy(&client_ctx->remote_addr, res->ai_addr, sizeof(client_ctx->remote_addr));
		client_ctx->remote_addr.sa_family = res->ai_family;
		freeaddrinfo(res);
#else
		goto cleanup;
#endif /* CONFIG_LWM2M_DNS_SUPPORT */
	}

	/* set port */
	if (client_ctx->remote_addr.sa_family == AF_INET6) {
		net_sin6(&client_ctx->remote_addr)->sin6_port = htons(parser.port);
	} else if (client_ctx->remote_addr.sa_family == AF_INET) {
		net_sin(&client_ctx->remote_addr)->sin_port = htons(parser.port);
	} else {
		ret = -EPROTONOSUPPORT;
	}

cleanup:
	/* restore host separator */
	url[off + len] = tmp;
	return ret;
}

static int lwm2m_transport_udp_open(struct lwm2m_ctx *client_ctx)
{
	if (client_ctx->sock_fd < 0) {
		if (IS_ENABLED(CONFIG_LWM2M_DTLS_SUPPORT) && client_ctx->use_dtls) {
			client_ctx->sock_fd = socket(client_ctx->remote_addr.sa_family, SOCK_DGRAM,
						     IPPROTO_DTLS_1_2);
		} else {
			client_ctx->sock_fd =
				socket(client_ctx->remote_addr.sa_family, SOCK_DGRAM, IPPROTO_UDP);
		}

		if (client_ctx->sock_fd < 0) {
			LOG_ERR("Failed to create socket: %d", errno);
			return -errno;
		}

		if (lwm2m_sock_table_update(client_ctx)) {
			return lwm2m_sock_table_add(client_ctx);
		}
	}

	return 0;
}

static int lwm2m_transport_udp_start(struct lwm2m_ctx *client_ctx)
{
	socklen_t addr_len;
	int flags;
	int ret;

#if defined(CONFIG_LWM2M_DTLS_SUPPORT)
	uint8_t tmp;
	int peer_verify =
#if defined(CONFIG_LWM2M_DTLS_VERIFY)
		TLS_PEER_VERIFY_REQUIRED;
#else
		TLS_PEER_VERIFY_OPTIONAL;
#endif

	if (client_ctx->load_credentials) {
		ret = client_ctx->load_credentials(client_ctx);
		if (ret < 0) {
			return ret;
		}
	}
#if defined(CONFIG_TLS_CREDENTIALS)
	else {
		ret = load_tls_credential(client_ctx, 3, TLS_CREDENTIAL_PSK_ID);
		if (ret < 0) {
			return ret;
		}

		ret = load_tls_credential(client_ctx, 5, TLS_CREDENTIAL_PSK);
		if (ret < 0) {
			return ret;
		}
	}
#endif /* CONFIG_TLS_CREDENTIALS */
#endif /* CONFIG_LWM2M_DTLS_SUPPORT */

	ret = lwm2m_transport_udp_open(client_ctx);
	if (ret) {
		return ret;
	}

#if defined(CONFIG_LWM2M_DTLS_SUPPORT)
	if (client_ctx->use_dtls) {
		sec_tag_t tls_tag_list[] = {
			client_ctx->tls_tag,
		};

		ret = setsockopt(client_ctx->sock_fd, SOL_TLS, TLS_SEC_TAG_LIST, tls_tag_list,
				 sizeof(tls_tag_list));
		if (ret < 0) {
			ret = -errno;
			LOG_ERR("Failed to set TLS_SEC_TAG_LIST option: %d", ret);
			goto error;
		}

		if (IS_ENABLED(CONFIG_LWM2M_TLS_SESSION_CACHING)) {
			int session_cache = TLS_SESSION_CACHE_ENABLED;

			ret = setsockopt(client_ctx->sock_fd, SOL_TLS, TLS_SESSION_CACHE,
					 &session_cache, sizeof(session_cache));
			if (ret < 0) {
				ret = -errno;
				LOG_ERR("Failed to set TLS_SESSION_CACHE option: %d", errno);
				goto error;
			}
		}

		ret = setsockopt(client_ctx->sock_fd, SOL_TLS, TLS_PEER_VERIFY, &peer_verify,
				 sizeof(peer_verify));
		if (ret < 0) {
			ret = -errno;
			LOG_ERR("Failed to set TLS_PEER_VERIFY option: %d", errno);
			goto error;
		}

client_ctx->hostname_verify = true;
		if (client_ctx->hostname_verify && (client_ctx->desthostname != NULL)) {
			/** store character at len position */
			tmp = client_ctx->desthostname[client_ctx->desthostnamelen];

			/** change it to '\0' to pass to socket*/
			client_ctx->desthostname[client_ctx->desthostnamelen] = '\0';

			/** mbedtls ignores length */
			ret = setsockopt(client_ctx->sock_fd, SOL_TLS, TLS_HOSTNAME,
					 client_ctx->desthostname, client_ctx->desthostnamelen);

			/** restore character */
			client_ctx->desthostname[client_ctx->desthostnamelen] = tmp;
			if (ret < 0) {
				ret = -errno;
				LOG_ERR("Failed to set TLS_HOSTNAME option: %d", ret);
				goto error;
			}
		}

		/*
		 * If we're using PSK credentials for DTLS, limit the list of ciphers to just those
		 * that support PSK.
		 */
		if (client_ctx->load_credentials == NULL) {
			filter_cipher_list(client_ctx->sock_fd);
		}
	}
#endif /* CONFIG_LWM2M_DTLS_SUPPORT */
	if ((client_ctx->remote_addr).sa_family == AF_INET) {
		addr_len = sizeof(struct sockaddr_in);
	} else if ((client_ctx->remote_addr).sa_family == AF_INET6) {
		addr_len = sizeof(struct sockaddr_in6);
	} else {
		lwm2m_engine_stop(client_ctx);
		return -EPROTONOSUPPORT;
	}

	if (connect(client_ctx->sock_fd, &client_ctx->remote_addr, addr_len) < 0) {
		ret = -errno;
		LOG_ERR("Cannot connect UDP (%d)", ret);
		goto error;
	}

	flags = fcntl(client_ctx->sock_fd, F_GETFL, 0);
	if (flags == -1) {
		ret = -errno;
		LOG_ERR("fcntl(F_GETFL) failed (%d)", ret);
		goto error;
	}
	ret = fcntl(client_ctx->sock_fd, F_SETFL, flags | O_NONBLOCK);
	if (ret == -1) {
		ret = -errno;
		LOG_ERR("fcntl(F_SETFL) failed (%d)", ret);
		goto error;
	}

	LOG_INF("Connected, sock id %d", client_ctx->sock_fd);
	return 0;
error:
	lwm2m_engine_stop(client_ctx);
	return ret;
}

static int lwm2m_transport_udp_suspend(struct lwm2m_ctx *client_ctx, bool should_close)
{
	int ret = 0;

	if (should_close) {
		if (client_ctx->sock_fd >= 0) {
			ret = close(client_ctx->sock_fd);
			if (ret) {
				LOG_ERR("Failed to close socket: %d", errno);
				ret = -errno;
				return ret;
			}

			client_ctx->connection_suspended = true;
#if defined(CONFIG_LWM2M_QUEUE_MODE_ENABLED)
			/* Enable Queue mode buffer store */
			client_ctx->buffer_client_messages = true;
#endif

			client_ctx->sock_fd = -1;
			lwm2m_sock_table_update(client_ctx);
		}
	} else {
		if (client_ctx->sock_fd >= 0 && !client_ctx->connection_suspended) {
			int socket_temp_id = client_ctx->sock_fd;

			client_ctx->connection_suspended = true;
#if defined(CONFIG_LWM2M_QUEUE_MODE_ENABLED)
			/* Enable Queue mode buffer store */
			client_ctx->buffer_client_messages = true;
#endif

			client_ctx->sock_fd = -1;
			lwm2m_sock_table_update(client_ctx);

			client_ctx->sock_fd = socket_temp_id;
		}
	}

	return ret;
}

static int lwm2m_transport_udp_resume(struct lwm2m_ctx *client_ctx)
{
	int ret;

	if (client_ctx->connection_suspended) {
		lwm2m_transport_udp_suspend(client_ctx, true);
		client_ctx->connection_suspended = false;

		ret = lwm2m_transport_udp_open(client_ctx);
		if (ret) {
			return ret;
		}

		if (!client_ctx->use_dtls) {
			return 0;
		}

		LOG_DBG("Resume suspended connection");
		return lwm2m_transport_udp_start(client_ctx);
	}

	return 0;
}

static int lwm2m_transport_udp_close(struct lwm2m_ctx *client_ctx)
{
	int sock_fd = client_ctx->sock_fd;

	lwm2m_sock_table_del(client_ctx);

	client_ctx->sock_fd = -1;
	if (sock_fd >= 0) {
		return close(sock_fd);
	}

	return 0;
}

static int lwm2m_transport_udp_send(struct lwm2m_ctx *client_ctx, const uint8_t *data,
				    uint32_t datalen)
{
	return send(client_ctx->sock_fd, data, datalen, 0);
}

static int lwm2m_transport_udp_recv(struct lwm2m_ctx *client_ctx)
{
	static uint8_t in_buf[NET_IPV6_MTU];
	socklen_t from_addr_len;
	ssize_t len;
	static struct sockaddr from_addr;

	from_addr_len = sizeof(from_addr);
	len = recvfrom(client_ctx->sock_fd, in_buf, sizeof(in_buf) - 1, 0, &from_addr,
		       &from_addr_len);

	if (len < 0) {
		if (errno == EAGAIN || errno == EWOULDBLOCK) {
			return -errno;
		}

		LOG_ERR("Error reading response: %d", errno);
		if (client_ctx->fault_cb != NULL) {
			client_ctx->fault_cb(client_ctx, errno);
		}
		return -errno;
	}

	if (len == 0) {
		LOG_ERR("Zero length recv");
		return 0;
	}

	in_buf[len] = 0U;
	lwm2m_coap_receive(client_ctx, in_buf, len, &from_addr);

	return 0;
}

static int lwm2m_transport_udp_is_connected(struct lwm2m_ctx *client_ctx)
{
	/*
	 * UDP is connectionless, but this could be changed in the future to
	 * only return true when the underlying network interface is ready.
	 */
	return true;
}

static void lwm2m_transport_udp_tx_pending(struct lwm2m_ctx *client_ctx, bool pending)
{
	/* nothing to do */
}

static char *lwm2m_transport_udp_print_addr(struct lwm2m_ctx *client_ctx,
					    const struct sockaddr *addr)
{
	static char buf[NET_IPV6_ADDR_LEN];

	if (addr->sa_family == AF_INET6) {
		return net_addr_ntop(AF_INET6, &net_sin6(addr)->sin6_addr, buf, sizeof(buf));
	}

	if (addr->sa_family == AF_INET) {
		return net_addr_ntop(AF_INET, &net_sin(addr)->sin_addr, buf, sizeof(buf));
	}

	LOG_ERR("Unknown IP address family:%d", addr->sa_family);
	strcpy(buf, "unk");
	return buf;
}
