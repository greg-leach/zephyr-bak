/*
 * Copyright (c) 2019 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_DOMAIN modem_hl7800
#define LOG_LEVEL CONFIG_MODEM_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_DOMAIN);

#include <kernel.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <stdlib.h>
#include <ctype.h>
#include <errno.h>
#include <zephyr.h>
#include <gpio.h>
#include <device.h>
#include <init.h>
#include <stdlib.h>
#include <power.h>

#include <net/net_if.h>
#include <net/net_context.h>
#include <net/net_offload.h>
#include <net/net_pkt.h>
#include <net/dns_resolve.h>
#if defined(CONFIG_NET_IPV6)
#include "ipv6.h"
#endif
#if defined(CONFIG_NET_IPV4)
#include "ipv4.h"
#endif
#if defined(CONFIG_NET_UDP)
#include "udp_internal.h"
#endif

#include <drivers/modem/modem_receiver.h>
#include <drivers/modem/hl7800.h>

#define PREFIXED_SWITCH_CASE_RETURN_STRING(prefix, val)                        \
	case prefix##_##val: {                                                 \
		return #val;                                                   \
	}

/* Uncomment the #define below to enable a hexdump of all incoming
 * data from the modem receiver
 */
/* #define HL7800_ENABLE_VERBOSE_MODEM_RECV_HEXDUMP 1 */

#define HL7800_LOG_UNHANDLED_RX_MSGS 1

#ifndef LOG_LEVEL_OFF
#define LOG_LEVEL_OFF (LOG_LEVEL_DBG + 1)
#endif

#define HL7800_RX_LOCK_LOG_LEVEL LOG_LEVEL_OFF
#define HL7800_TX_LOCK_LOG_LEVEL LOG_LEVEL_OFF
#define HL7800_WAKE_IO_LOG_LEVEL LOG_LEVEL_OFF
#define HL7800_IO_LOG_LEVEL LOG_LEVEL_OFF

#if ((LOG_LEVEL == LOG_LEVEL_DBG) &&                                           \
     defined(CONFIG_MODEM_HL7800_LOW_POWER_MODE))
#define PRINT_AWAKE_MSG LOG_WRN("awake")
#define PRINT_NOT_AWAKE_MSG LOG_WRN("NOT awake")
#else
#define PRINT_AWAKE_MSG
#define PRINT_NOT_AWAKE_MSG
#endif

enum tcp_notif {
	HL7800_TCP_NET_ERR,
	HL7800_TCP_NO_SOCKS,
	HL7800_TCP_MEM,
	HL7800_TCP_DNS,
	HL7800_TCP_DISCON,
	HL7800_TCP_CONN,
	HL7800_TCP_ERR,
	HL7800_TCP_CLIENT_REQ,
	HL7800_TCP_DATA_SND,
	HL7800_TCP_ID,
	HL7800_TCP_RUNNING,
	HL7800_TCP_ALL_USED,
	HL7800_TCP_TIMEOUT,
	HL7800_TCP_SSL_CONN,
	HL7800_TCP_SSL_INIT
};

enum udp_notif {
	HL7800_UDP_NET_ERR = 0,
	HL7800_UDP_NO_SOCKS = 1,
	HL7800_UDP_MEM = 2,
	HL7800_UDP_DNS = 3,
	HL7800_UDP_CONN = 5,
	HL7800_UDP_ERR = 6,
	HL7800_UDP_DATA_SND = 8, /* this matches TCP_DATA_SND */
	HL7800_UDP_ID = 9,
	HL7800_UDP_RUNNING = 10,
	HL7800_UDP_ALL_USED = 11
};

enum socket_state { SOCK_IDLE, SOCK_RX, SOCK_TX };

struct mdm_control_pinconfig {
	char *dev_name;
	u32_t pin;
	int config;
	int sleep_config;
};

#define PINCONFIG(name_, pin_, config_, sleep_config_)                         \
	{                                                                      \
		.dev_name = name_, .pin = pin_, .config = config_,             \
		.sleep_config = sleep_config_                                  \
	}

/* pin settings */
enum mdm_control_pins {
	MDM_RESET = 0,
	MDM_WAKE,
	MDM_PWR_ON,
	MDM_FAST_SHUTD,
	MDM_UART_DTR,
	MDM_VGPIO,
	MDM_UART_DSR,
	MDM_UART_TX,
	MDM_UART_RTS,
	MDM_UART_RX,
	MDM_UART_CTS,
	MDM_GPIO6,
	MAX_MDM_CONTROL_PINS,
};

enum net_operator_status { NO_OPERATOR, REGISTERED };

static const struct mdm_control_pinconfig pinconfig[] = {
	/* MDM_RESET */
	PINCONFIG(DT_SWI_HL7800_0_MDM_RESET_GPIOS_CONTROLLER,
		  DT_SWI_HL7800_0_MDM_RESET_GPIOS_PIN,
		  (GPIO_DIR_OUT | GPIO_DS_DISCONNECT_HIGH),
		  (GPIO_DIR_OUT | GPIO_DS_DISCONNECT_HIGH)),

	/* MDM_WAKE */
	PINCONFIG(DT_SWI_HL7800_0_MDM_WAKE_GPIOS_CONTROLLER,
		  DT_SWI_HL7800_0_MDM_WAKE_GPIOS_PIN,
		  (GPIO_DIR_OUT | GPIO_DS_DISCONNECT_LOW),
		  (GPIO_DIR_OUT | GPIO_DS_DISCONNECT_LOW)),

	/* MDM_PWR_ON */
	PINCONFIG(DT_SWI_HL7800_0_MDM_PWR_ON_GPIOS_CONTROLLER,
		  DT_SWI_HL7800_0_MDM_PWR_ON_GPIOS_PIN,
		  (GPIO_DIR_OUT | GPIO_DS_DISCONNECT_HIGH),
		  (GPIO_DIR_OUT | GPIO_DS_DISCONNECT_HIGH)),

	/* MDM_FAST_SHUTD */
	PINCONFIG(DT_SWI_HL7800_0_MDM_FAST_SHUTD_GPIOS_CONTROLLER,
		  DT_SWI_HL7800_0_MDM_FAST_SHUTD_GPIOS_PIN,
		  (GPIO_DIR_OUT | GPIO_DS_DISCONNECT_HIGH),
		  (GPIO_DIR_OUT | GPIO_DS_DISCONNECT_HIGH)),

	/* MDM_UART_DTR */
	PINCONFIG(DT_SWI_HL7800_0_MDM_UART_DTR_GPIOS_CONTROLLER,
		  DT_SWI_HL7800_0_MDM_UART_DTR_GPIOS_PIN,
		  (GPIO_DIR_OUT | GPIO_DS_DISCONNECT_HIGH),
		  (GPIO_DIR_OUT | GPIO_DS_DISCONNECT_HIGH)),

	/* MDM_VGPIO */
	PINCONFIG(DT_SWI_HL7800_0_MDM_VGPIO_GPIOS_CONTROLLER,
		  DT_SWI_HL7800_0_MDM_VGPIO_GPIOS_PIN,
		  (GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
		   GPIO_INT_DOUBLE_EDGE),
		  (GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
		   GPIO_INT_DOUBLE_EDGE)),

	/* MDM_UART_DSR */
	PINCONFIG(DT_SWI_HL7800_0_MDM_UART_DSR_GPIOS_CONTROLLER,
		  DT_SWI_HL7800_0_MDM_UART_DSR_GPIOS_PIN,
		  (GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
		   GPIO_INT_DOUBLE_EDGE),
		  (GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
		   GPIO_INT_DOUBLE_EDGE)),

	/* MDM_UART_TX */
	PINCONFIG(DT_SWI_HL7800_0_MDM_UART_TX_GPIOS_CONTROLLER,
		  DT_SWI_HL7800_0_MDM_UART_TX_GPIOS_PIN, GPIO_DIR_OUT,
		  (GPIO_DIR_IN | GPIO_PUD_PULL_DOWN)),

	/* MDM_UART_RTS */
	PINCONFIG(DT_SWI_HL7800_0_MDM_UART_RTS_GPIOS_CONTROLLER,
		  DT_SWI_HL7800_0_MDM_UART_RTS_GPIOS_PIN, GPIO_DIR_OUT,
		  (GPIO_DIR_IN | GPIO_PUD_PULL_DOWN)),

	/* MDM_UART_RX */
	PINCONFIG(DT_SWI_HL7800_0_MDM_UART_RX_GPIOS_CONTROLLER,
		  DT_SWI_HL7800_0_MDM_UART_RX_GPIOS_PIN, GPIO_DIR_IN,
		  (GPIO_DIR_IN | GPIO_PUD_PULL_DOWN)),

	/* MDM_UART_CTS */
	PINCONFIG(DT_SWI_HL7800_0_MDM_UART_CTS_GPIOS_CONTROLLER,
		  DT_SWI_HL7800_0_MDM_UART_CTS_GPIOS_PIN,
		  (GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
		   GPIO_INT_DOUBLE_EDGE),
		  (GPIO_DIR_IN | GPIO_PUD_PULL_DOWN | GPIO_INT | GPIO_INT_EDGE |
		   GPIO_INT_DOUBLE_EDGE)),

	/* MDM_GPIO6 */
	PINCONFIG(DT_SWI_HL7800_0_MDM_GPIO6_GPIOS_CONTROLLER,
		  DT_SWI_HL7800_0_MDM_GPIO6_GPIOS_PIN,
		  (GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
		   GPIO_INT_DOUBLE_EDGE),
		  (GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
		   GPIO_INT_DOUBLE_EDGE)),
};

#define MDM_UART_DEV_NAME DT_SWI_HL7800_0_BUS_NAME

#define MDM_WAKE_ASSERTED 1 /* Asserted keeps the module awake */
#define MDM_WAKE_NOT_ASSERTED 0
#define MDM_RESET_ASSERTED 0
#define MDM_RESET_NOT_ASSERTED 1
#define MDM_PWR_ON_ASSERTED 0
#define MDM_PWR_ON_NOT_ASSERTED 1
#define MDM_FAST_SHUTD_ASSERTED 0
#define MDM_FAST_SHUTD_NOT_ASSERTED 1
#define MDM_UART_DTR_ASSERTED 0 /* Asserted keeps the module awake */
#define MDM_UART_DTR_NOT_ASSERTED 1

#define MDM_SEND_OK_ENABLED 0
#define MDM_SEND_OK_DISABLED 1

#define MDM_CMD_SEND_TIMEOUT K_SECONDS(5)
#define MDM_IP_SEND_RX_TIMEOUT K_SECONDS(60)
#define MDM_SOCK_NOTIF_DELAY K_MSEC(150)
#define MDM_CMD_CONN_TIMEOUT K_SECONDS(31)

#define MDM_MAX_DATA_LENGTH 1500
#define MDM_MTU 1500

#define MDM_RECV_MAX_BUF 30
#define MDM_RECV_BUF_SIZE 128
#define MDM_HANDLER_MATCH_MAX_LEN 100

#define MDM_MAX_SOCKETS 6

#define BUF_ALLOC_TIMEOUT K_SECONDS(1)

#define SIZE_OF_NUL 1

#define SIZE_WITHOUT_NUL(v) (sizeof(v) - SIZE_OF_NUL)

#define CMD_HANDLER(cmd_, cb_)                                                 \
	{                                                                      \
		.cmd = cmd_, .cmd_len = (u16_t)sizeof(cmd_) - 1,               \
		.func = on_cmd_##cb_                                           \
	}

#define MDM_MANUFACTURER_LENGTH 16
#define MDM_MODEL_LENGTH 7
#define MDM_SN_RESPONSE_LENGTH MDM_HL7800_SERIAL_NUMBER_SIZE + 7
#define MDM_NETWORK_STATUS_LENGTH 45

#define MDM_TOP_BAND_SIZE 4
#define MDM_MIDDLE_BAND_SIZE 8
#define MDM_BOTTOM_BAND_SIZE 8
#define MDM_TOP_BAND_START_POSITION 2
#define MDM_MIDDLE_BAND_START_POSITION 6
#define MDM_BOTTOM_BAND_START_POSITION 14

#define MDM_DEFAULT_AT_CMD_RETRIES 3
#define MDM_WAKEUP_TIME K_SECONDS(12)
#define MDM_BOOT_TIME K_SECONDS(12)

#define MDM_WAIT_FOR_DATA_TIME K_MSEC(50)
#define MDM_RESET_LOW_TIME K_MSEC(50)
#define MDM_RESET_HIGH_TIME K_MSEC(10)
#define MDM_WAIT_FOR_DATA_RETRIES 3

#define RSSI_TIMEOUT_SECS 30
#define RSSI_UNKNOWN -999

#define DNS_WORK_DELAY_SECS 1
#define IFACE_WORK_DELAY K_MSEC(500)
#define WAIT_FOR_KSUP_RETRIES 5

#define PROFILE_LINE_1                                                         \
	"E1 Q0 V1 X4 &C1 &D1 &R1 &S0 +IFC=2,2 &K3 +IPR=115200 +FCLASS0\r\n"
#define PROFILE_LINE_2                                                         \
	"S00:255 S01:255 S03:255 S04:255 S05:255 S07:255 S08:255 S10:255\r\n"

#define SETUP_GPRS_CONNECTION_CMD "AT+KCNXCFG=1,\"GPRS\",\"\""

#define MAX_PROFILE_LINE_LENGTH                                                \
	MAX(sizeof(PROFILE_LINE_1), sizeof(PROFILE_LINE_2))

/* The ? can be a + or - */
const char TIME_STRING_FORMAT[] = "\"yy/MM/dd,hh:mm:ss?zz\"";
#define TIME_STRING_DIGIT_STRLEN 2
#define TIME_STRING_SEPARATOR_STRLEN 1
#define TIME_STRING_PLUS_MINUS_INDEX (6 * 3)
#define TIME_STRING_FIRST_SEPARATOR_INDEX 0
#define TIME_STRING_FIRST_DIGIT_INDEX 1
#define TIME_STRING_TO_TM_STRUCT_YEAR_OFFSET (2000 - 1900)

/* Time structure min, max */
#define TM_YEAR_RANGE 0, 99
#define TM_MONTH_RANGE_PLUS_1 1, 12
#define TM_DAY_RANGE 1, 31
#define TM_HOUR_RANGE 0, 23
#define TM_MIN_RANGE 0, 59
#define TM_SEC_RANGE 0, 60 /* leap second */
#define QUARTER_HOUR_RANGE 0, 96
#define SECONDS_PER_QUARTER_HOUR (15 * 60)

#define SEND_AT_CMD_ONCE_EXPECT_OK(c)                                          \
	do {                                                                   \
		ret = send_at_cmd(NULL, (c), MDM_CMD_SEND_TIMEOUT, 0, false);  \
		if (ret < 0) {                                                 \
			LOG_ERR("%s result:%d", (c), ret);                     \
			goto error;                                            \
		}                                                              \
	} while (0)

#define SEND_AT_CMD_IGNORE_ERROR(c)                                            \
	do {                                                                   \
		ret = send_at_cmd(NULL, (c), MDM_CMD_SEND_TIMEOUT, 0, false);  \
		if (ret < 0) {                                                 \
			LOG_ERR("%s result:%d", (c), ret);                     \
		}                                                              \
	} while (0)

#define SEND_AT_CMD_EXPECT_OK(c)                                               \
	do {                                                                   \
		ret = send_at_cmd(NULL, (c), MDM_CMD_SEND_TIMEOUT,             \
				  MDM_DEFAULT_AT_CMD_RETRIES, false);          \
		if (ret < 0) {                                                 \
			LOG_ERR("%s result:%d", (c), ret);                     \
			goto error;                                            \
		}                                                              \
	} while (0)

/* Complex has "no_id_resp" set to true because the sending command
* is the command used to process the respone */
#define SEND_COMPLEX_AT_CMD(c)                                                 \
	do {                                                                   \
		ret = send_at_cmd(NULL, (c), MDM_CMD_SEND_TIMEOUT,             \
				  MDM_DEFAULT_AT_CMD_RETRIES, true);           \
		if (ret < 0) {                                                 \
			LOG_ERR("%s result:%d", (c), ret);                     \
			goto error;                                            \
		}                                                              \
	} while (0)

NET_BUF_POOL_DEFINE(mdm_recv_pool, MDM_RECV_MAX_BUF, MDM_RECV_BUF_SIZE, 0,
		    NULL);

static u8_t mdm_recv_buf[MDM_MAX_DATA_LENGTH];

static K_SEM_DEFINE(hl7800_RX_lock_sem, 1, 1);
static K_SEM_DEFINE(hl7800_TX_lock_sem, 1, 1);

/* RX thread structures */
K_THREAD_STACK_DEFINE(hl7800_rx_stack, CONFIG_MODEM_HL7800_RX_STACK_SIZE);
struct k_thread hl7800_rx_thread;
#define RX_THREAD_PRIORITY K_PRIO_COOP(7)

/* RX thread work queue */
K_THREAD_STACK_DEFINE(hl7800_workq_stack,
		      CONFIG_MODEM_HL7800_RX_WORKQ_STACK_SIZE);
static struct k_work_q hl7800_workq;
#define WORKQ_PRIORITY K_PRIO_COOP(7)

static const char EOF_PATTERN[] = "--EOF--Pattern--";
static const char CONNECT_STRING[] = "CONNECT";
static const char OK_STRING[] = "OK";

struct hl7800_socket {
	struct net_context *context;
	sa_family_t family;
	enum net_sock_type type;
	enum net_ip_protocol ip_proto;
	struct sockaddr src;
	struct sockaddr dst;

	bool created;
	int socket_id;
	int rx_size;
	bool error;
	int error_val;
	enum socket_state state;

	/** semaphore */
	struct k_sem sock_send_sem;

	/** socket callbacks */
	struct k_work recv_cb_work;
	struct k_work rx_data_work;
	struct k_delayed_work notif_work;
	net_context_recv_cb_t recv_cb;
	struct net_pkt *recv_pkt;
	void *recv_user_data;
};

#define NO_ID_RESP_CMD_MAX_LENGTH 32

struct hl7800_iface_ctx {
	struct net_if *iface;
	u8_t mac_addr[6];
	struct in_addr ipv4Addr, subnet, gateway, dns;
	bool restarting;
	bool initialized;
	bool wait_for_KSUP;
	u32_t wait_for_KSUP_tries;
	bool reconfig_GPRS;
	char dns_string[sizeof("###.###.###.###")];
	char no_id_resp_cmd[NO_ID_RESP_CMD_MAX_LENGTH];
	bool search_no_id_resp;

	/* GPIO PORT devices */
	struct device *gpio_port_dev[MAX_MDM_CONTROL_PINS];
	struct gpio_callback mdm_vgpio_cb;
	struct gpio_callback mdm_uart_dsr_cb;
	struct gpio_callback mdm_gpio6_cb;
	struct gpio_callback mdm_uart_cts_cb;
	u32_t vgpio_state;
	u32_t dsr_state;
	u32_t gpio6_state;
	u32_t cts_state;

	/* RX specific attributes */
	struct mdm_receiver_context mdm_ctx;

	/* socket data */
	struct hl7800_socket sockets[MDM_MAX_SOCKETS];
	int last_socket_id;
	int last_error;

	/* semaphores */
	struct k_sem response_sem;
	struct k_sem mdm_awake;

	/* RSSI work */
	struct k_delayed_work rssi_query_work;

	/* work */
	struct k_delayed_work iface_status_work;
	struct k_delayed_work dns_work;
	struct k_work mdm_vgpio_work;
	struct k_delayed_work mdm_reset_work;

	/* modem info */
	/* NOTE: make sure length is +1 for null char */
	char mdm_manufacturer[MDM_MANUFACTURER_LENGTH];
	char mdm_model[MDM_MODEL_LENGTH];
	char mdm_revision[MDM_HL7800_REVISION_MAX_SIZE];
	char mdm_imei[MDM_HL7800_IMEI_SIZE];
	char mdm_sn[MDM_HL7800_SERIAL_NUMBER_SIZE];
	char mdm_network_status[MDM_NETWORK_STATUS_LENGTH];
	char mdm_iccid[MDM_HL7800_ICCID_SIZE];
	u8_t mdm_startup_state;
	enum mdm_hl7800_radio_mode mdm_rat;
	u16_t mdm_bands_top;
	u32_t mdm_bands_middle;
	u32_t mdm_bands_bottom;
	s32_t mdm_sinr;
	bool mdm_echo_is_on;
	struct mdm_hl7800_apn mdm_apn;
	bool mdm_startup_reporting_on;

	/* modem state */
	bool allowSleep;
	enum mdm_hl7800_sleep_state sleep_state;
	enum mdm_hl7800_network_state network_state;
	enum net_operator_status operator_status;
	void (*event_callback)(enum mdm_hl7800_event event, void *event_data);
#ifdef CONFIG_NEWLIB_LIBC
	struct tm local_time;
	s32_t local_time_offset;
#endif
	bool local_time_valid;
};

struct cmd_handler {
	const char *cmd;
	u16_t cmd_len;
	bool (*func)(struct net_buf **buf, u16_t len);
};

static struct hl7800_iface_ctx ictx;

static size_t hl7800_read_rx(struct net_buf **buf);
static char *get_network_state_string(enum mdm_hl7800_network_state state);
static char *get_startup_state_string(enum mdm_hl7800_startup_state state);
static char *get_sleep_state_string(enum mdm_hl7800_sleep_state state);
static void set_network_state(enum mdm_hl7800_network_state state);
static void set_startup_state(enum mdm_hl7800_startup_state state);
static void set_sleep_state(enum mdm_hl7800_sleep_state state);
static void generate_network_state_event(void);
static void generate_startup_state_event(void);
static void generate_sleep_state_event(void);
static int modem_boot_handler(char *reason);
static void mdm_vgpio_work_cb(struct k_work *item);
static void mdm_reset_work_callback(struct k_work *item);
static bool is_network_ready(void);

#ifdef CONFIG_NEWLIB_LIBC
static bool convert_time_string_to_struct(struct tm *tm, s32_t *offset,
					  char *time_string);
#endif

#ifdef CONFIG_MODEM_HL7800_LOW_POWER_MODE
static bool is_cmd_ready()
{
	gpio_pin_read(ictx.gpio_port_dev[MDM_VGPIO], pinconfig[MDM_VGPIO].pin,
		      &ictx.vgpio_state);

	gpio_pin_read(ictx.gpio_port_dev[MDM_GPIO6], pinconfig[MDM_GPIO6].pin,
		      &ictx.gpio6_state);

	gpio_pin_read(ictx.gpio_port_dev[MDM_UART_CTS],
		      pinconfig[MDM_UART_CTS].pin, &ictx.cts_state);

	return ictx.vgpio_state && ictx.gpio6_state && !ictx.cts_state;
}
#endif

/**
 * The definition of awake is that the HL7800 is ready to receive AT commands successfully
 */
static void check_hl7800_awake()
{
#ifdef CONFIG_MODEM_HL7800_LOW_POWER_MODE
	bool isCmdRdy = is_cmd_ready();
	if (isCmdRdy && (ictx.sleep_state != HL7800_SLEEP_STATE_AWAKE) &&
	    !ictx.allowSleep && !ictx.wait_for_KSUP) {
		PRINT_AWAKE_MSG;
		set_sleep_state(HL7800_SLEEP_STATE_AWAKE);
		k_sem_give(&ictx.mdm_awake);
	} else if (!isCmdRdy && ictx.sleep_state == HL7800_SLEEP_STATE_AWAKE &&
		   ictx.allowSleep) {
		PRINT_NOT_AWAKE_MSG;
		set_sleep_state(HL7800_SLEEP_STATE_ASLEEP);
		k_sem_reset(&ictx.mdm_awake);
	}
#endif
}

static int hl7800_RX_lock(void)
{
	Z_LOG(HL7800_RX_LOCK_LOG_LEVEL, "Locking RX [%p]...", k_current_get());
	int rc = k_sem_take(&hl7800_RX_lock_sem, K_FOREVER);
	if (rc != 0) {
		LOG_ERR("Unable to lock hl7800 (%d)", rc);
	} else {
		Z_LOG(HL7800_RX_LOCK_LOG_LEVEL, "Locked RX [%p]",
		      k_current_get());
	}

	return rc;
}

static void hl7800_RX_unlock(void)
{
	Z_LOG(HL7800_RX_LOCK_LOG_LEVEL, "UNLocking RX [%p]...",
	      k_current_get());
	k_sem_give(&hl7800_RX_lock_sem);
	Z_LOG(HL7800_RX_LOCK_LOG_LEVEL, "UNLocked RX [%p]", k_current_get());
}

static bool hl7800_RX_locked(void)
{
	if (k_sem_count_get(&hl7800_RX_lock_sem) == 0) {
		return true;
	} else {
		return false;
	}
}

static int hl7800_TX_lock(void)
{
	Z_LOG(HL7800_TX_LOCK_LOG_LEVEL, "Locking TX [%p]...", k_current_get());
	int rc = k_sem_take(&hl7800_TX_lock_sem, K_FOREVER);
	if (rc != 0) {
		LOG_ERR("Unable to lock hl7800 (%d)", rc);
	} else {
		Z_LOG(HL7800_TX_LOCK_LOG_LEVEL, "Locked TX [%p]",
		      k_current_get());
	}

	return rc;
}

static void hl7800_TX_unlock(void)
{
	Z_LOG(HL7800_TX_LOCK_LOG_LEVEL, "UNLocking TX [%p]...",
	      k_current_get());
	k_sem_give(&hl7800_TX_lock_sem);
	Z_LOG(HL7800_TX_LOCK_LOG_LEVEL, "UNLocked TX [%p]", k_current_get());
}

static bool hl7800_TX_locked(void)
{
	if (k_sem_count_get(&hl7800_TX_lock_sem) == 0) {
		return true;
	} else {
		return false;
	}
}

static void hl7800_lock(void)
{
	hl7800_TX_lock();
	hl7800_RX_lock();
}

static void hl7800_unlock(void)
{
	hl7800_RX_unlock();
	hl7800_TX_unlock();
}

static struct hl7800_socket *socket_get(void)
{
	int i;
	struct hl7800_socket *sock = NULL;

	for (i = 0; i < MDM_MAX_SOCKETS; i++) {
		if (!ictx.sockets[i].context) {
			sock = &ictx.sockets[i];
			break;
		}
	}

	return sock;
}

static struct hl7800_socket *socket_from_id(int socket_id)
{
	int i;
	struct hl7800_socket *sock = NULL;

	if (socket_id < 1) {
		return NULL;
	}

	for (i = 0; i < MDM_MAX_SOCKETS; i++) {
		if (ictx.sockets[i].socket_id == socket_id) {
			sock = &ictx.sockets[i];
			break;
		}
	}

	return sock;
}

static void socket_put(struct hl7800_socket *sock)
{
	if (!sock) {
		return;
	}

	sock->context = NULL;
	sock->socket_id = -1;
	sock->created = false;
	sock->error = false;
	sock->error_val = -1;
	sock->rx_size = 0;
	sock->state = SOCK_IDLE;
	(void)memset(&sock->src, 0, sizeof(struct sockaddr));
	(void)memset(&sock->dst, 0, sizeof(struct sockaddr));
}

char *hl7800_sprint_ip_addr(const struct sockaddr *addr)
{
	static char buf[NET_IPV6_ADDR_LEN];

#if defined(CONFIG_NET_IPV6)
	if (addr->sa_family == AF_INET6) {
		return net_addr_ntop(AF_INET6, &net_sin6(addr)->sin6_addr, buf,
				     sizeof(buf));
	} else
#endif
#if defined(CONFIG_NET_IPV4)
		if (addr->sa_family == AF_INET) {
		return net_addr_ntop(AF_INET, &net_sin(addr)->sin_addr, buf,
				     sizeof(buf));
	} else
#endif
	{
		LOG_ERR("Unknown IP address family:%d", addr->sa_family);
		return NULL;
	}
}

static void modem_assert_wake(bool assert)
{
	if (assert) {
		Z_LOG(HL7800_WAKE_IO_LOG_LEVEL, "MDM_WAKE_PIN -> ASSERTED");
		gpio_pin_write(ictx.gpio_port_dev[MDM_WAKE],
			       pinconfig[MDM_WAKE].pin, MDM_WAKE_ASSERTED);
	} else {
		Z_LOG(HL7800_WAKE_IO_LOG_LEVEL, "MDM_WAKE_PIN -> NOT_ASSERTED");
		gpio_pin_write(ictx.gpio_port_dev[MDM_WAKE],
			       pinconfig[MDM_WAKE].pin, MDM_WAKE_NOT_ASSERTED);
	}
}

static void modem_assert_pwr_on(bool assert)
{
	if (assert) {
		Z_LOG(HL7800_IO_LOG_LEVEL, "MDM_PWR_ON -> ASSERTED");
		gpio_pin_write(ictx.gpio_port_dev[MDM_PWR_ON],
			       pinconfig[MDM_PWR_ON].pin, MDM_PWR_ON_ASSERTED);
	} else {
		Z_LOG(HL7800_IO_LOG_LEVEL, "MDM_PWR_ON -> NOT_ASSERTED");
		gpio_pin_write(ictx.gpio_port_dev[MDM_PWR_ON],
			       pinconfig[MDM_PWR_ON].pin,
			       MDM_PWR_ON_NOT_ASSERTED);
	}
}

static void modem_assert_fast_shutd(bool assert)
{
	if (assert) {
		Z_LOG(HL7800_IO_LOG_LEVEL, "MDM_FAST_SHUTD -> ASSERTED");
		gpio_pin_write(ictx.gpio_port_dev[MDM_FAST_SHUTD],
			       pinconfig[MDM_FAST_SHUTD].pin,
			       MDM_FAST_SHUTD_ASSERTED);
	} else {
		Z_LOG(HL7800_IO_LOG_LEVEL, "MDM_FAST_SHUTD -> NOT_ASSERTED");
		gpio_pin_write(ictx.gpio_port_dev[MDM_FAST_SHUTD],
			       pinconfig[MDM_FAST_SHUTD].pin,
			       MDM_FAST_SHUTD_NOT_ASSERTED);
	}
}

static void modem_assert_uart_dtr(bool assert)
{
	if (assert) {
		Z_LOG(HL7800_WAKE_IO_LOG_LEVEL, "MDM_UART_DTR -> ASSERTED");
		gpio_pin_write(ictx.gpio_port_dev[MDM_UART_DTR],
			       pinconfig[MDM_UART_DTR].pin,
			       MDM_UART_DTR_ASSERTED);
	} else {
		Z_LOG(HL7800_WAKE_IO_LOG_LEVEL, "MDM_UART_DTR -> NOT_ASSERTED");
		gpio_pin_write(ictx.gpio_port_dev[MDM_UART_DTR],
			       pinconfig[MDM_UART_DTR].pin,
			       MDM_UART_DTR_NOT_ASSERTED);
	}
}

static void allow_sleep(bool allow)
{
#ifdef CONFIG_MODEM_HL7800_LOW_POWER_MODE
	if (allow) {
		LOG_INF("Allow sleep");
		ictx.allowSleep = true;
		modem_assert_wake(false);
		modem_assert_uart_dtr(false);
	} else {
		LOG_INF("Keep awake");
		ictx.allowSleep = false;
		modem_assert_wake(true);
		modem_assert_uart_dtr(true);
	}
#endif
}

static void event_handler(enum mdm_hl7800_event event, void *event_data)
{
	if (ictx.event_callback != NULL) {
		ictx.event_callback(event, event_data);
	}
}

void mdm_hl7800_get_signal_quality(int *rsrp, int *sinr)
{
	*rsrp = ictx.mdm_ctx.data_rssi;
	*sinr = ictx.mdm_sinr;
}

void mdm_hl7800_wakeup(bool wakeup)
{
	allow_sleep(!wakeup);
}

/* Send an AT command with a series of response handlers */
static int send_at_cmd(struct hl7800_socket *sock, const u8_t *data,
		       int timeout, int retries, bool no_id_resp)
{
	int ret = 0;

	ictx.last_error = 0;

	do {
		if (!sock) {
			k_sem_reset(&ictx.response_sem);
			ictx.last_socket_id = 0;
		} else {
			k_sem_reset(&sock->sock_send_sem);
			ictx.last_socket_id = sock->socket_id;
		}
		if (no_id_resp) {
			strncpy(ictx.no_id_resp_cmd, data,
				sizeof(ictx.no_id_resp_cmd));
			ictx.search_no_id_resp = true;
		}

		LOG_DBG("OUT: [%s]", log_strdup(data));
		mdm_receiver_send(&ictx.mdm_ctx, data, strlen(data));
		mdm_receiver_send(&ictx.mdm_ctx, "\r", 1);

		if (timeout == K_NO_WAIT) {
			goto done;
		}

		if (!sock) {
			ret = k_sem_take(&ictx.response_sem, timeout);
		} else {
			ret = k_sem_take(&sock->sock_send_sem, timeout);
		}

		if (ret == 0) {
			ret = ictx.last_error;
		} else if (ret == -EAGAIN) {
			ret = -ETIMEDOUT;
		}

		retries--;
		if (retries < 0) {
			retries = 0;
		}
	} while (ret != 0 && retries > 0);
done:
	ictx.search_no_id_resp = false;
	return ret;
}

static int wakeup_hl7800(void)
{
#ifdef CONFIG_MODEM_HL7800_LOW_POWER_MODE
	int ret;
	allow_sleep(false);
	if (!is_cmd_ready()) {
		LOG_DBG("Waiting to wakeup");
		ret = k_sem_take(&ictx.mdm_awake, MDM_WAKEUP_TIME);
		if (ret) {
			LOG_ERR("Err waiting for wakeup: %d", ret);
		}
	}
#endif
	return 0;
}

s32_t mdm_hl7800_send_at_cmd(const u8_t *data)
{
	int ret;

	if (!data) {
		return -EINVAL;
	}

	hl7800_lock();

	bool goBackToSleep = ictx.allowSleep;
	wakeup_hl7800();
	ictx.last_socket_id = 0;
	ret = send_at_cmd(NULL, data, MDM_CMD_SEND_TIMEOUT, 0, false);
	allow_sleep(goBackToSleep);
	hl7800_unlock();
	return ret;
}

/* The access point name (and username and password) are stored in the modem's
 * non-volatile memory.
 */
s32_t mdm_hl7800_update_apn(char *access_point_name)
{
	int ret = -EINVAL;
	char cmd_string[MDM_HL7800_APN_CMD_MAX_SIZE];

	hl7800_lock();
	wakeup_hl7800();
	bool goBackToSleep = ictx.allowSleep;
	ictx.last_socket_id = 0;

	/* PDP Context */
	memset(cmd_string, 0, MDM_HL7800_APN_CMD_MAX_SIZE);
	strncat(cmd_string, "AT+CGDCONT=1,\"IP\",\"",
		MDM_HL7800_APN_CMD_MAX_STRLEN);
	strncat(cmd_string, access_point_name, MDM_HL7800_APN_CMD_MAX_STRLEN);
	strncat(cmd_string, "\"", MDM_HL7800_APN_CMD_MAX_STRLEN);
	SEND_AT_CMD_ONCE_EXPECT_OK(cmd_string);

error:
	allow_sleep(goBackToSleep);
	hl7800_unlock();
	if (ret >= 0) {
		/* After a reset the APN will be re-read from the modem
		 * and an event will be generated.
		 */
		k_delayed_work_submit_to_queue(&hl7800_workq,
					       &ictx.mdm_reset_work, K_NO_WAIT);
	}
	return ret;
}

bool mdm_hl7800_valid_rat(u8_t value)
{
	if ((value == MDM_RAT_CAT_M1) || (value == MDM_RAT_CAT_NB1)) {
		return true;
	}
	return false;
}

s32_t mdm_hl7800_update_rat(enum mdm_hl7800_radio_mode value)
{
	int ret = -EINVAL;
	bool goBackToSleep;

	if (value == ictx.mdm_rat) {
		// The set command will fail (in the modem) if the RAT isn't different.
		return 0;
	} else if (!mdm_hl7800_valid_rat(value)) {
		return ret;
	}

	hl7800_lock();
	/* store the current sleep state because wakeup will modify this */
	goBackToSleep = ictx.allowSleep;
	wakeup_hl7800();
	ictx.last_socket_id = 0;

	if (value == MDM_RAT_CAT_M1) {
		SEND_AT_CMD_ONCE_EXPECT_OK("AT+KSRAT=0");
	} else { // MDM_RAT_CAT_NB1
		SEND_AT_CMD_ONCE_EXPECT_OK("AT+KSRAT=1");
	}

error:
	if (ret >= 0) {
		/* Changing the RAT causes the modem to reset. */
		ret = modem_boot_handler("RAT changed");
	}
	/* restore the sleep state */
	allow_sleep(goBackToSleep);
	hl7800_unlock();
	return ret;
}

#ifdef CONFIG_NEWLIB_LIBC
s32_t mdm_hl7800_get_local_time(struct tm *tm, s32_t *offset)
{
	int ret;

	ictx.local_time_valid = false;

	hl7800_lock();
	bool goBackToSleep = ictx.allowSleep;
	wakeup_hl7800();
	ictx.last_socket_id = 0;
	ret = send_at_cmd(NULL, "AT+CCLK?", MDM_CMD_SEND_TIMEOUT, 0, false);
	allow_sleep(goBackToSleep);
	if (ictx.local_time_valid) {
		memcpy(tm, &ictx.local_time, sizeof(struct tm));
		memcpy(offset, &ictx.local_time_offset, sizeof(*offset));
	} else {
		ret = -EIO;
	}
	hl7800_unlock();
	return ret;
}
#endif

void mdm_hl7800_generate_status_events(void)
{
	hl7800_lock();
	generate_startup_state_event();
	generate_network_state_event();
	generate_sleep_state_event();
	event_handler(HL7800_EVENT_RSSI, &ictx.mdm_ctx.data_rssi);
	event_handler(HL7800_EVENT_SINR, &ictx.mdm_sinr);
	event_handler(HL7800_EVENT_APN_UPDATE, &ictx.mdm_apn);
	event_handler(HL7800_EVENT_RAT, &ictx.mdm_rat);
	hl7800_unlock();
}

static int send_data(struct hl7800_socket *sock, struct net_pkt *pkt)
{
	int ret, bufSize;
	struct net_buf *frag;
	char dstAddr[sizeof("###.###.###.###")];
	size_t sendLen, actualSendLen;
	sendLen = 0, actualSendLen = 0;

	if (sock->type == SOCK_STREAM) {
		bufSize = sizeof("AT+KTCPSND=##,####");
	} else {
		bufSize =
			sizeof("AT+KUDPSND=##,\"###.###.###.###\",#####,####");
	}

	char buf[bufSize];

	if (!sock) {
		return -EINVAL;
	}

	ictx.last_error = 0;
	sock->state = SOCK_TX;

	frag = pkt->frags;
	sendLen = net_buf_frags_len(frag);
	/* start sending data */
	k_sem_reset(&sock->sock_send_sem);
	if (sock->type == SOCK_STREAM) {
		snprintk(buf, sizeof(buf), "AT+KTCPSND=%d,%u", sock->socket_id,
			 sendLen);
	} else {
		if (!net_addr_ntop(sock->family, &net_sin(&sock->dst)->sin_addr,
				   dstAddr, sizeof(dstAddr))) {
			LOG_ERR("Invalid dst addr");
			return -EINVAL;
		}
		snprintk(buf, sizeof(buf), "AT+KUDPSND=%d,\"%s\",%u,%u",
			 sock->socket_id, dstAddr,
			 net_sin(&sock->dst)->sin_port, sendLen);
	}
	send_at_cmd(sock, buf, K_NO_WAIT, 0, false);

	/* wait for CONNECT  or error */
	ret = k_sem_take(&sock->sock_send_sem, MDM_IP_SEND_RX_TIMEOUT);
	if (ret) {
		LOG_ERR("Err waiting for CONNECT (%d)", ret);
		goto done;
	}
	/* check for error */
	if (ictx.last_error != 0) {
		ret = ictx.last_error;
		LOG_ERR("AT+K**PSND (%d)", ret);
		goto done;
	}

	/* Loop through packet data and send */
	while (frag) {
		actualSendLen += frag->len;
		mdm_receiver_send(&ictx.mdm_ctx, frag->data, frag->len);
		frag = frag->frags;
	}
	if (actualSendLen != sendLen) {
		LOG_WRN("AT+K**PSND act: %d exp: %d", actualSendLen, sendLen);
	}
	LOG_DBG("Sent %u bytes", actualSendLen);

	/* Send EOF pattern to terminate data */
	k_sem_reset(&sock->sock_send_sem);
	mdm_receiver_send(&ictx.mdm_ctx, EOF_PATTERN, strlen(EOF_PATTERN));
	ret = k_sem_take(&sock->sock_send_sem, MDM_CMD_SEND_TIMEOUT);
	if (ret == 0) {
		ret = ictx.last_error;
	} else if (ret == -EAGAIN) {
		ret = -ETIMEDOUT;
	}
done:
	sock->state = SOCK_IDLE;
	return ret;
}

/*** NET_BUF HELPERS ***/

static bool is_crlf(u8_t c)
{
	if (c == '\n' || c == '\r') {
		return true;
	} else {
		return false;
	}
}

static void net_buf_skipcrlf(struct net_buf **buf)
{
	/* chop off any /n or /r */
	while (*buf && is_crlf(*(*buf)->data)) {
		net_buf_pull_u8(*buf);
		if (!(*buf)->len) {
			*buf = net_buf_frag_del(NULL, *buf);
		}
	}
}

static u16_t net_buf_findcrlf(struct net_buf *buf, struct net_buf **frag)
{
	u16_t len = 0U, pos = 0U;

	while (buf && !is_crlf(*(buf->data + pos))) {
		if (pos + 1 >= buf->len) {
			len += buf->len;
			buf = buf->frags;
			pos = 0U;
		} else {
			pos++;
		}
	}

	if (buf && is_crlf(*(buf->data + pos))) {
		len += pos;
		*frag = buf;
		return len;
	}

	return 0;
}

static u8_t net_buf_get_u8(struct net_buf **buf)
{
	u8_t val = net_buf_pull_u8(*buf);
	if (!(*buf)->len) {
		*buf = net_buf_frag_del(NULL, *buf);
	}
	return val;
}

static u32_t net_buf_remove(struct net_buf **buf, u32_t len)
{
	u32_t toRemove;
	u32_t removed = 0;

	while (*buf && len > 0) {
		toRemove = (*buf)->len;
		if (toRemove > len) {
			toRemove = len;
		}
		net_buf_pull(*buf, toRemove);
		removed += toRemove;
		len -= toRemove;
		if (!(*buf)->len) {
			*buf = net_buf_frag_del(NULL, *buf);
		}
	}
	return removed;
}

/*** UDP / TCP Helper Function ***/

/* Setup IP header data to be used by some network applications.
 * While much is dummy data, some fields such as dst, port and family are
 * important.
 * Return the IP + protocol header length.
 */
static int pkt_setup_ip_data(struct net_pkt *pkt, struct hl7800_socket *sock)
{
	int hdr_len = 0;
	u16_t src_port = 0U, dst_port = 0U;

#if defined(CONFIG_NET_IPV6)
	if (net_pkt_family(pkt) == AF_INET6) {
		if (net_ipv6_create(
			    pkt,
			    &((struct sockaddr_in6 *)&sock->dst)->sin6_addr,
			    &((struct sockaddr_in6 *)&sock->src)->sin6_addr)) {
			return -1;
		}
		src_port = ntohs(net_sin6(&sock->src)->sin6_port);
		dst_port = ntohs(net_sin6(&sock->dst)->sin6_port);

		hdr_len = sizeof(struct net_ipv6_hdr);
	}
#endif
#if defined(CONFIG_NET_IPV4)
	if (net_pkt_family(pkt) == AF_INET) {
		if (net_ipv4_create(
			    pkt, &((struct sockaddr_in *)&sock->dst)->sin_addr,
			    &((struct sockaddr_in *)&sock->src)->sin_addr)) {
			return -1;
		}
		src_port = ntohs(net_sin(&sock->src)->sin_port);
		dst_port = ntohs(net_sin(&sock->dst)->sin_port);

		hdr_len = sizeof(struct net_ipv4_hdr);
	}
#endif

#if defined(CONFIG_NET_UDP)
	if (sock->ip_proto == IPPROTO_UDP) {
		if (net_udp_create(pkt, dst_port, src_port)) {
			return -1;
		}

		hdr_len += NET_UDPH_LEN;
	}
#endif
#if defined(CONFIG_NET_TCP)
	if (sock->ip_proto == IPPROTO_TCP) {
		NET_PKT_DATA_ACCESS_DEFINE(tcp_access, struct net_tcp_hdr);
		struct net_tcp_hdr *tcp;

		tcp = (struct net_tcp_hdr *)net_pkt_get_data(pkt, &tcp_access);
		if (!tcp) {
			return -1;
		}

		(void)memset(tcp, 0, NET_TCPH_LEN);

		/* Setup TCP header */
		tcp->src_port = src_port;
		tcp->dst_port = dst_port;

		if (net_pkt_set_data(pkt, &tcp_access)) {
			return -1;
		}

		hdr_len += NET_TCPH_LEN;
	}
#endif /* CONFIG_NET_TCP */

	return hdr_len;
}

/*** MODEM RESPONSE HANDLERS ***/

static u32_t wait_for_modem_data(struct net_buf **buf, u32_t currentLen,
				 u32_t expectedLen)
{
	u32_t waitForDataTries = 0;
	while ((currentLen < expectedLen) &&
	       (waitForDataTries < MDM_WAIT_FOR_DATA_RETRIES)) {
		LOG_DBG("cur:%d, exp:%d", currentLen, expectedLen);
		k_sleep(MDM_WAIT_FOR_DATA_TIME);
		currentLen += hl7800_read_rx(buf);
		waitForDataTries++;
	}

	return currentLen;
}

static uint32_t wait_for_modem_data_and_newline(struct net_buf **buf,
						u32_t currentLen,
						u32_t expectedLen)
{
	return wait_for_modem_data(buf, currentLen,
				   (expectedLen + strlen("\r\n")));
}

/* Handler: AT+CGMI */
static bool on_cmd_atcmdinfo_manufacturer(struct net_buf **buf, u16_t len)
{
	struct net_buf *frag = NULL;
	size_t out_len;
	int len_no_null = MDM_MANUFACTURER_LENGTH - 1;

	/* make sure revision data is received
	*  waiting for: Sierra Wireless\r\n
	*/
	wait_for_modem_data_and_newline(buf, net_buf_frags_len(*buf),
					MDM_MANUFACTURER_LENGTH);

	frag = NULL;
	len = net_buf_findcrlf(*buf, &frag);
	if (!frag) {
		LOG_ERR("Unable to find mfg end");
		goto done;
	}
	if (len < len_no_null) {
		LOG_WRN("mfg too short (len:%d)", len);
	} else if (len > len_no_null) {
		LOG_WRN("mfg too long (len:%d)", len);
		len = MDM_MANUFACTURER_LENGTH;
	}

	out_len = net_buf_linearize(ictx.mdm_manufacturer,
				    sizeof(ictx.mdm_manufacturer) - 1, *buf, 0,
				    len);
	ictx.mdm_manufacturer[out_len] = 0;
	LOG_INF("Manufacturer: %s", ictx.mdm_manufacturer);
done:
	return true;
}

/* Handler: AT+CGMM */
static bool on_cmd_atcmdinfo_model(struct net_buf **buf, u16_t len)
{
	struct net_buf *frag = NULL;
	size_t out_len;
	int len_no_null = MDM_MODEL_LENGTH - 1;

	/* make sure model data is received
	*  waiting for: HL7800\r\n
	*/
	wait_for_modem_data_and_newline(buf, net_buf_frags_len(*buf),
					MDM_MODEL_LENGTH);

	frag = NULL;
	len = net_buf_findcrlf(*buf, &frag);
	if (!frag) {
		LOG_ERR("Unable to find model end");
		goto done;
	}
	if (len < len_no_null) {
		LOG_WRN("model too short (len:%d)", len);
	} else if (len > len_no_null) {
		LOG_WRN("model too long (len:%d)", len);
		len = MDM_MODEL_LENGTH;
	}

	out_len = net_buf_linearize(ictx.mdm_model, sizeof(ictx.mdm_model) - 1,
				    *buf, 0, len);
	ictx.mdm_model[out_len] = 0;
	LOG_INF("Model: %s", ictx.mdm_model);
done:
	return true;
}

/* Handler: AT+CGMR */
static bool on_cmd_atcmdinfo_revision(struct net_buf **buf, u16_t len)
{
	struct net_buf *frag = NULL;
	size_t out_len;

	/* make sure revision data is received
	*  waiting for something like: AHL7800.1.2.3.1.20171211\r\n
	*/
	wait_for_modem_data_and_newline(buf, net_buf_frags_len(*buf),
					MDM_HL7800_REVISION_MAX_SIZE);

	frag = NULL;
	len = net_buf_findcrlf(*buf, &frag);
	if (!frag) {
		LOG_ERR("Unable to find rev end");
		goto done;
	}
	if (len == 0) {
		LOG_WRN("revision not found");
	} else if (len > MDM_HL7800_REVISION_MAX_STRLEN) {
		LOG_WRN("revision too long (len:%d)", len);
		len = MDM_HL7800_REVISION_MAX_STRLEN;
	}

	out_len = net_buf_linearize(
		ictx.mdm_revision, sizeof(ictx.mdm_revision) - 1, *buf, 0, len);
	ictx.mdm_revision[out_len] = 0;
	LOG_INF("Revision: %s", ictx.mdm_revision);
done:
	return true;
}

/* Handler: AT+CGSN */
static bool on_cmd_atcmdinfo_imei(struct net_buf **buf, u16_t len)
{
	struct net_buf *frag = NULL;
	size_t out_len;

	/* make sure IMEI data is received
	*  waiting for: ###############\r\n
	*/
	wait_for_modem_data_and_newline(buf, net_buf_frags_len(*buf),
					MDM_HL7800_IMEI_SIZE);

	frag = NULL;
	len = net_buf_findcrlf(*buf, &frag);
	if (!frag) {
		LOG_ERR("Unable to find IMEI end");
		goto done;
	}
	if (len < MDM_HL7800_IMEI_STRLEN) {
		LOG_WRN("IMEI too short (len:%d)", len);
	} else if (len > MDM_HL7800_IMEI_STRLEN) {
		LOG_WRN("IMEI too long (len:%d)", len);
		len = MDM_HL7800_IMEI_STRLEN;
	}

	out_len = net_buf_linearize(ictx.mdm_imei, sizeof(ictx.mdm_imei) - 1,
				    *buf, 0, len);
	ictx.mdm_imei[out_len] = 0;

	LOG_INF("IMEI: %s", ictx.mdm_imei);
done:
	return true;
}

/* Handler: +CCID: <ICCID> */
static bool on_cmd_atcmdinfo_iccid(struct net_buf **buf, u16_t len)
{
	struct net_buf *frag = NULL;
	size_t out_len;

	/* make sure ICCID data is received
	*  waiting for: <ICCID>\r\n
	*/
	wait_for_modem_data_and_newline(buf, net_buf_frags_len(*buf),
					MDM_HL7800_ICCID_SIZE);

	frag = NULL;
	len = net_buf_findcrlf(*buf, &frag);
	if (!frag) {
		LOG_ERR("Unable to find ICCID end");
		goto done;
	}
	if (len < MDM_HL7800_ICCID_STRLEN) {
		LOG_WRN("ICCID too short (len:%d)", len);
	} else if (len > MDM_HL7800_ICCID_STRLEN) {
		LOG_WRN("ICCID too long (len:%d)", len);
		len = MDM_HL7800_ICCID_STRLEN;
	}

	out_len = net_buf_linearize(ictx.mdm_iccid, MDM_HL7800_ICCID_STRLEN,
				    *buf, 0, len);
	ictx.mdm_iccid[out_len] = 0;

	LOG_INF("ICCID: %s", ictx.mdm_iccid);
done:
	return true;
}

static void dns_work_cb(struct k_work *work)
{
#ifdef CONFIG_DNS_RESOLVER
	int ret;
	/* set new DNS addr in DNS resolver */
	struct dns_resolve_context *dnsCtx = dns_resolve_get_default();
	dns_resolve_close(dnsCtx);
	const char *dns_servers_str[] = { ictx.dns_string };
	ret = dns_resolve_init(dnsCtx, dns_servers_str, NULL);
	if (ret < 0) {
		LOG_ERR("dns_resolve_init fail (%d)", ret);
		return;
	}
#endif
}

char *mdm_hl7800_get_iccid(void)
{
	return ictx.mdm_iccid;
}

char *mdm_hl7800_get_sn(void)
{
	return ictx.mdm_sn;
}

/* Handler: +CGCONTRDP: <cid>,<bearer_id>,<apn>,<local_addr and subnet_mask>,
*			<gw_addr>,<DNS_prim_addr>,<DNS_sec_addr> */
static bool on_cmd_atcmdinfo_ipaddr(struct net_buf **buf, u16_t len)
{
	int ret;
	int numDelims = 7;
	char *delims[numDelims];
	size_t out_len;
	char value[len + 1];
	char *searchStart, *addrStart, *smStart, *gwStart, *dnsStart;
	struct in_addr newIpv4Addr;

	out_len = net_buf_linearize(value, sizeof(value), *buf, 0, len);
	value[out_len] = 0;
	searchStart = value;

	/* find all delimiters (,) */
	for (int i = 0; i < numDelims; i++) {
		delims[i] = strchr(searchStart, ',');
		if (!delims[i]) {
			LOG_ERR("Could not find delim %d, val: %s", i,
				log_strdup(value));
			return true;
		}
		/* Start next search after current delim location */
		searchStart = delims[i] + 1;
	}

	/* Find start of subnet mask */
	addrStart = delims[2] + 1;
	numDelims = 4;
	searchStart = addrStart;
	for (int i = 0; i < numDelims; i++) {
		smStart = strchr(searchStart, '.');
		if (!smStart) {
			LOG_ERR("Could not find submask start");
			return true;
		}
		/* Start next search after current delim location */
		searchStart = smStart + 1;
	}

	/* get new IPv4 addr */
	int ipv4Len = smStart - addrStart;
	char ipv4AddrStr[ipv4Len + 1];
	strncpy(ipv4AddrStr, addrStart, ipv4Len);
	ipv4AddrStr[ipv4Len] = 0;
	ret = net_addr_pton(AF_INET, ipv4AddrStr, &newIpv4Addr);
	if (ret < 0) {
		LOG_ERR("Invalid IPv4 addr");
		return true;
	}

	/* move past the '.' */
	smStart += 1;
	/* store new subnet mask */
	int snLen = delims[3] - smStart;
	char smStr[snLen + 1];
	strncpy(smStr, smStart, snLen);
	smStr[snLen] = 0;
	ret = net_addr_pton(AF_INET, smStr, &ictx.subnet);
	if (ret < 0) {
		LOG_ERR("Invalid subnet");
		return true;
	}

	/* store new gateway */
	gwStart = delims[3] + 1;
	int gwLen = delims[4] - gwStart;
	char gwStr[gwLen + 1];
	strncpy(gwStr, gwStart, gwLen);
	gwStr[gwLen] = 0;
	ret = net_addr_pton(AF_INET, gwStr, &ictx.gateway);
	if (ret < 0) {
		LOG_ERR("Invalid gateway");
		return true;
	}

	/* store new dns */
	dnsStart = delims[4] + 1;
	int dnsLen = delims[5] - dnsStart;
	strncpy(ictx.dns_string, dnsStart, dnsLen);
	ictx.dns_string[dnsLen] = 0;
	ret = net_addr_pton(AF_INET, ictx.dns_string, &ictx.dns);
	if (ret < 0) {
		LOG_ERR("Invalid dns");
		return true;
	}

	if (ictx.iface) {
		/* remove the current IPv4 addr before adding a new one.
		*  We dont care if it is successful or not.
		*/
		net_if_ipv4_addr_rm(ictx.iface, &ictx.ipv4Addr);

		if (!net_if_ipv4_addr_add(ictx.iface, &newIpv4Addr,
					  NET_ADDR_DHCP, 0)) {
			LOG_ERR("Cannot set iface IPv4 addr");
			return true;
		}

		net_if_ipv4_set_netmask(ictx.iface, &ictx.subnet);
		net_if_ipv4_set_gw(ictx.iface, &ictx.gateway);

		/* store the new IP addr */
		net_ipaddr_copy(&ictx.ipv4Addr, &newIpv4Addr);

		/* start DNS update work */
		s32_t delay = K_NO_WAIT;
		if (!ictx.initialized) {
			/* Delay this in case the network
			*  stack is still starting up */
			delay = K_SECONDS(DNS_WORK_DELAY_SECS);
		}
		k_delayed_work_submit_to_queue(&hl7800_workq, &ictx.dns_work,
					       delay);
	} else {
		LOG_ERR("iface NULL");
	}

	/* TODO: IPv6 addr present, store it */
	return true;
}

/* Handler: +COPS: <mode>[,<format>,<oper>[,<AcT>]] */
static bool on_cmd_atcmdinfo_operator_status(struct net_buf **buf, u16_t len)
{
	size_t out_len;
	char value[len + 1];
	int numDelims = 2;
	char *delims[numDelims];
	char *searchStart;

	out_len = net_buf_linearize(value, sizeof(value), *buf, 0, len);
	value[out_len] = 0;
	LOG_INF("Operator: %s", log_strdup(value));

	if (len == 1) {
		/* only mode was returned, there is no operator info */
		ictx.operator_status = NO_OPERATOR;
		goto done;
	}

	searchStart = value;

	/* find all delimiters (,) */
	for (int i = 0; i < numDelims; i++) {
		delims[i] = strchr(searchStart, ',');
		if (!delims[i]) {
			LOG_ERR("Could not find delim %d, val: %s", i,
				log_strdup(value));
			goto done;
		}
		/* Start next search after current delim location */
		searchStart = delims[i] + 1;
	}

	/* we found both delimiters, that means we have an operator */
	ictx.operator_status = REGISTERED;
done:
	return true;
}

/* Handler: +KGSN: T5640400011101 */
static bool on_cmd_atcmdinfo_serial_number(struct net_buf **buf, u16_t len)
{
	struct net_buf *frag = NULL;
	char value[MDM_SN_RESPONSE_LENGTH];
	size_t out_len;
	int sn_len;
	char *valStart;

	/* make sure SN# data is received.
	*  we are waiting for: +KGSN: ##############\r\n
	*/
	wait_for_modem_data(buf, net_buf_frags_len(*buf),
			    MDM_SN_RESPONSE_LENGTH);

	frag = NULL;
	len = net_buf_findcrlf(*buf, &frag);
	if (!frag) {
		LOG_ERR("Unable to find sn end");
		goto done;
	}

	/* get msg data */
	out_len = net_buf_linearize(value, sizeof(value), *buf, 0, len);
	value[out_len] = 0;

	/* find ':' */
	valStart = strchr(value, ':');
	if (!valStart) {
		LOG_ERR("Unable to find sn ':'");
		goto done;
	}
	/* Remove ": " chars */
	valStart += 2;

	sn_len = len - (valStart - value);
	if (sn_len < MDM_HL7800_SERIAL_NUMBER_STRLEN) {
		LOG_WRN("sn too short (len:%d)", sn_len);
	} else if (sn_len > MDM_HL7800_SERIAL_NUMBER_STRLEN) {
		LOG_WRN("sn too long (len:%d)", sn_len);
		sn_len = MDM_HL7800_SERIAL_NUMBER_STRLEN;
	}

	strncpy(ictx.mdm_sn, valStart, sn_len);
	ictx.mdm_sn[sn_len] = 0;
	LOG_INF("Serial #: %s", ictx.mdm_sn);
done:
	return true;
}

/* Handler: +KSRAT: # */
static bool on_cmd_radio_tech_status(struct net_buf **buf, u16_t len)
{
	size_t out_len;
	char value[len + 1];

	out_len = net_buf_linearize(value, sizeof(value) - 1, *buf, 0, len);
	value[out_len] = 0;
	ictx.mdm_rat = strtol(value, NULL, 10);
	event_handler(HL7800_EVENT_RAT, &ictx.mdm_rat);

	return true;
}

/* Handler: +KBNDCFG: #,####################### */
static bool on_cmd_radio_band_configuration(struct net_buf **buf, u16_t len)
{
	size_t out_len;
	char value[len + 1];
	char nTmp[sizeof("#########")];

	out_len = net_buf_linearize(value, sizeof(value) - 1, *buf, 0, len);
	value[out_len] = 0;

	if (value[0] != (ictx.mdm_rat == MDM_RAT_CAT_M1 ? '0' : '1')) {
		//Invalid RAT
		return true;
	} else if (strlen(value) < sizeof("#,###################")) {
		//String size too short
		return true;
	}

	memcpy(nTmp, &value[MDM_TOP_BAND_START_POSITION], MDM_TOP_BAND_SIZE);
	nTmp[MDM_TOP_BAND_SIZE] = 0;
	ictx.mdm_bands_top = strtoul(nTmp, NULL, 16);

	memcpy(nTmp, &value[MDM_MIDDLE_BAND_START_POSITION],
	       MDM_MIDDLE_BAND_SIZE);
	nTmp[MDM_MIDDLE_BAND_SIZE] = 0;
	ictx.mdm_bands_middle = strtoul(nTmp, NULL, 16);

	memcpy(nTmp, &value[MDM_BOTTOM_BAND_START_POSITION],
	       MDM_BOTTOM_BAND_SIZE);
	nTmp[MDM_BOTTOM_BAND_SIZE] = 0;
	ictx.mdm_bands_bottom = strtoul(nTmp, NULL, 16);

	LOG_INF("Current band configuration: %04x %08x %08x",
		ictx.mdm_bands_top, ictx.mdm_bands_middle,
		ictx.mdm_bands_bottom);

	return true;
}

static char *get_startup_state_string(enum mdm_hl7800_startup_state state)
{
	// clang-format off
	switch (state) {
		PREFIXED_SWITCH_CASE_RETURN_STRING(HL7800_STARTUP_STATE, READY);
		PREFIXED_SWITCH_CASE_RETURN_STRING(HL7800_STARTUP_STATE, WAITING_FOR_ACCESS_CODE);
		PREFIXED_SWITCH_CASE_RETURN_STRING(HL7800_STARTUP_STATE, SIM_NOT_PRESENT);
		PREFIXED_SWITCH_CASE_RETURN_STRING(HL7800_STARTUP_STATE, SIMLOCK);
		PREFIXED_SWITCH_CASE_RETURN_STRING(HL7800_STARTUP_STATE, UNRECOVERABLE_ERROR);
		PREFIXED_SWITCH_CASE_RETURN_STRING(HL7800_STARTUP_STATE, UNKNOWN);
		PREFIXED_SWITCH_CASE_RETURN_STRING(HL7800_STARTUP_STATE, INACTIVE_SIM);
	default:
		return "UNKNOWN";
	}
	// clang-format on
}

static void set_startup_state(enum mdm_hl7800_startup_state state)
{
	ictx.mdm_startup_state = state;
	generate_startup_state_event();
}

static void generate_startup_state_event(void)
{
	struct mdm_hl7800_compound_event event;
	event.code = ictx.mdm_startup_state;
	event.string = get_startup_state_string(ictx.mdm_startup_state);
	LOG_INF("Startup State: %s", event.string);
	event_handler(HL7800_EVENT_STARTUP_STATE_CHANGE, &event);
}

static char *get_sleep_state_string(enum mdm_hl7800_sleep_state state)
{
	// clang-format off
	switch (state) {
		PREFIXED_SWITCH_CASE_RETURN_STRING(HL7800_SLEEP_STATE, UNINITIALIZED);
		PREFIXED_SWITCH_CASE_RETURN_STRING(HL7800_SLEEP_STATE, ASLEEP);
		PREFIXED_SWITCH_CASE_RETURN_STRING(HL7800_SLEEP_STATE, AWAKE);
	default:
		return "UNKNOWN";
	}
	// clang-format on
}

static void set_sleep_state(enum mdm_hl7800_sleep_state state)
{
	ictx.sleep_state = state;
	generate_sleep_state_event();
}

static void generate_sleep_state_event(void)
{
	struct mdm_hl7800_compound_event event;
	event.code = ictx.sleep_state;
	event.string = get_sleep_state_string(ictx.sleep_state);
	LOG_INF("Sleep State: %s", event.string);
	event_handler(HL7800_EVENT_SLEEP_STATE_CHANGE, &event);
}

/* Handler: +KSUP: # */
static bool on_cmd_startup_report(struct net_buf **buf, u16_t len)
{
	char value[len + SIZE_OF_NUL];
	memset(value, 0, sizeof(value));
	size_t out_len =
		net_buf_linearize(value, SIZE_WITHOUT_NUL(value), *buf, 0, len);
	if (out_len > 0) {
		set_startup_state(strtol(value, NULL, 10));
	} else {
		set_startup_state(HL7800_STARTUP_STATE_UNKNOWN);
	}

	PRINT_AWAKE_MSG;
	ictx.wait_for_KSUP = false;
	ictx.mdm_startup_reporting_on = true;
	set_sleep_state(HL7800_SLEEP_STATE_AWAKE);
	k_sem_give(&ictx.mdm_awake);
	return true;
}

static bool profile_handler(struct net_buf **buf, u16_t len,
			    bool active_profile)
{
	// Prepare net buffer for this parser.
	net_buf_remove(buf, len);
	net_buf_skipcrlf(buf);

	u32_t size = wait_for_modem_data(buf, net_buf_frags_len(*buf),
					 sizeof(PROFILE_LINE_1));
	net_buf_skipcrlf(buf); // remove any \r\n that are in the front

	// Parse configuration data to determine if echo is on/off.
	int echo_state = -1;
	struct net_buf *frag = NULL;
	u16_t line_length = net_buf_findcrlf(*buf, &frag);
	if (line_length) {
		char line[MAX_PROFILE_LINE_LENGTH];
		memset(line, 0, sizeof(line));
		size_t output_length = net_buf_linearize(
			line, SIZE_WITHOUT_NUL(line), *buf, 0, line_length);
		Z_LOG(LOG_LEVEL_DBG, "length: %u: %s", line_length,
		      log_strdup(line));

		// Echo on off is the first thing on the line: E0, E1
		if (output_length >= SIZE_WITHOUT_NUL("E?")) {
			echo_state = (line[1] == '1') ? 1 : 0;
		}
	}
	Z_LOG(LOG_LEVEL_OFF, "echo: %d", echo_state);
	net_buf_remove(buf, line_length);
	net_buf_skipcrlf(buf);

	if (active_profile) {
		ictx.mdm_echo_is_on = (echo_state != 0);
	}

	// Discard next line.  This waits for the longest possible response even
	// though most registers won't have the value 0xFF.
	size = wait_for_modem_data(buf, net_buf_frags_len(*buf),
				   sizeof(PROFILE_LINE_2));
	net_buf_skipcrlf(buf);
	len = net_buf_findcrlf(*buf, &frag);
	net_buf_remove(buf, len);
	net_buf_skipcrlf(buf);

	return false;
}

static bool on_cmd_atcmdinfo_active_profile(struct net_buf **buf, u16_t len)
{
	return profile_handler(buf, len, true);
}

static bool on_cmd_atcmdinfo_stored_profile0(struct net_buf **buf, u16_t len)
{
	return profile_handler(buf, len, false);
}

static bool on_cmd_atcmdinfo_stored_profile1(struct net_buf **buf, u16_t len)
{
	return profile_handler(buf, len, false);
}

/* +WPPP: 1,1,"username","password" */
static bool on_cmd_atcmdinfo_pdp_authentication_cfg(struct net_buf **buf,
						    u16_t len)
{
	wait_for_modem_data_and_newline(buf, net_buf_frags_len(*buf),
					MDM_HL7800_APN_CMD_MAX_SIZE);

	struct net_buf *frag = NULL;
	u16_t line_length = net_buf_findcrlf(*buf, &frag);
	if (line_length) {
		char line[MDM_HL7800_APN_CMD_MAX_SIZE];
		memset(line, 0, sizeof(line));
		size_t output_length = net_buf_linearize(
			line, SIZE_WITHOUT_NUL(line), *buf, 0, line_length);
		Z_LOG(LOG_LEVEL_DBG, "length: %u: %s", line_length,
		      log_strdup(line));
		if (output_length > 0) {
			memset(ictx.mdm_apn.username, 0,
			       sizeof(ictx.mdm_apn.username));
			memset(ictx.mdm_apn.password, 0,
			       sizeof(ictx.mdm_apn.password));

			size_t i = 0;
			char *p = strchr(line, '"');
			if (p != NULL) {
				p += 1;
				i = 0;
				while ((p != NULL) && (*p != '"') &&
				       (i <
					MDM_HL7800_APN_USERNAME_MAX_STRLEN)) {
					ictx.mdm_apn.username[i++] = *p++;
				}
			}
			LOG_INF("APN Username: %s", ictx.mdm_apn.username);

			p = strchr(p + 1, '"');
			if (p != NULL) {
				p += 1;
				i = 0;
				while ((p != NULL) && (*p != '"') &&
				       (i <
					MDM_HL7800_APN_PASSWORD_MAX_STRLEN)) {
					ictx.mdm_apn.password[i++] = *p++;
				}
			}
			LOG_INF("APN Password: %s", ictx.mdm_apn.password);
		}
	}
	net_buf_remove(buf, line_length);
	net_buf_skipcrlf(buf);

	return false;
}

/* Only context 1 is used. Other contexts are unhandled.
 *
 * +CGDCONT: 1,"IP","access point name",,0,0,0,0,0,,0,,,,,
 */
static bool on_cmd_atcmdinfo_pdp_context(struct net_buf **buf, u16_t len)
{
	wait_for_modem_data_and_newline(buf, net_buf_frags_len(*buf),
					MDM_HL7800_APN_CMD_MAX_SIZE);

	struct net_buf *frag = NULL;
	u16_t line_length = net_buf_findcrlf(*buf, &frag);
	if (line_length) {
		char line[MDM_HL7800_APN_CMD_MAX_SIZE];
		memset(line, 0, sizeof(line));
		size_t output_length = net_buf_linearize(
			line, SIZE_WITHOUT_NUL(line), *buf, 0, line_length);
		Z_LOG(LOG_LEVEL_DBG, "length: %u: %s", line_length,
		      log_strdup(line));
		if (output_length > 0) {
			memset(ictx.mdm_apn.value, 0,
			       sizeof(ictx.mdm_apn.value));

			/* The name is after the 3rd " */
			char *p = strchr(line, '"');
			p = strchr(p + 1, '"');
			p = strchr(p + 1, '"');
			if (p != NULL) {
				p += 1;
				size_t i = 0;
				while ((p != NULL) && (*p != '"') &&
				       (i < MDM_HL7800_APN_MAX_STRLEN)) {
					ictx.mdm_apn.value[i++] = *p++;
				}
			}
			LOG_INF("APN: %s", ictx.mdm_apn.value);
		}
	}
	net_buf_remove(buf, line_length);
	net_buf_skipcrlf(buf);

	return false;
}

static int hl7800_query_rssi(void)
{
	int ret;
	ret = send_at_cmd(NULL, "AT+KCELLMEAS=0", MDM_CMD_SEND_TIMEOUT, 1,
			  false);
	if (ret < 0) {
		LOG_ERR("AT+KCELLMEAS ret:%d", ret);
	}
	return ret;
}

static void hl7800_start_rssi_work(void)
{
	k_delayed_work_submit_to_queue(&hl7800_workq, &ictx.rssi_query_work,
				       K_NO_WAIT);
}

static void hl7800_stop_rssi_work(void)
{
	k_delayed_work_cancel(&ictx.rssi_query_work);
}

static void hl7800_rssi_query_work(struct k_work *work)
{
	hl7800_lock();
	hl7800_query_rssi();
	hl7800_unlock();

	/* re-start RSSI query work */
	k_delayed_work_submit_to_queue(&hl7800_workq, &ictx.rssi_query_work,
				       K_SECONDS(RSSI_TIMEOUT_SECS));
}

static void iface_status_work_cb(struct k_work *work)
{
	int ret;
	hl7800_lock();

	if (!ictx.initialized && ictx.restarting) {
		LOG_DBG("Wait for driver init, process network state later");
		/* we are not ready to process this yet, try again later */
		k_delayed_work_submit_to_queue(&hl7800_workq,
					       &ictx.iface_status_work,
					       IFACE_WORK_DELAY);
		goto done;
	} else if (ictx.wait_for_KSUP &&
		   ictx.wait_for_KSUP_tries < WAIT_FOR_KSUP_RETRIES) {
		LOG_DBG("Wait for +KSUP before updating network state");
		ictx.wait_for_KSUP_tries++;
		/* we have not received +KSUP yet, lets wait more time to receive +KSUP */
		k_delayed_work_submit_to_queue(&hl7800_workq,
					       &ictx.iface_status_work,
					       IFACE_WORK_DELAY);
		goto done;
	} else if (ictx.wait_for_KSUP &&
		   ictx.wait_for_KSUP_tries >= WAIT_FOR_KSUP_RETRIES) {
		/* give up waiting for KSUP */
		LOG_DBG("Give up waiting for");
		ictx.wait_for_KSUP = false;
		check_hl7800_awake();
	}

	LOG_DBG("Updating network state...");

	/* Query operator selection */
	ret = send_at_cmd(NULL, "AT+COPS?", MDM_CMD_SEND_TIMEOUT, 0, false);
	if (ret < 0) {
		LOG_ERR("AT+COPS ret:%d", ret);
	}

	/* bring iface up/down */
	switch (ictx.network_state) {
	case HL7800_HOME_NETWORK:
	case HL7800_ROAMING:
		if (ictx.iface && !net_if_is_up(ictx.iface)) {
			LOG_DBG("HL7800 iface UP");
			net_if_up(ictx.iface);
		}
		break;
	case HL7800_OUT_OF_COVERAGE:
	default:
		if (ictx.iface && net_if_is_up(ictx.iface)) {
			LOG_DBG("HL7800 iface DOWN");
			net_if_down(ictx.iface);
			allow_sleep(true);
		}
		break;
	}

	if (ictx.iface && !net_if_is_up(ictx.iface)) {
		hl7800_stop_rssi_work();
	} else if (ictx.iface && net_if_is_up(ictx.iface)) {
		hl7800_start_rssi_work();

		/* get IP address info */
		ret = send_at_cmd(NULL, "AT+CGCONTRDP=1", MDM_CMD_SEND_TIMEOUT,
				  0, false);
		if (ret < 0) {
			LOG_ERR("AT+CGCONTRDP ret:%d", ret);
		}
	}
	LOG_DBG("Network state updated");
done:
	hl7800_unlock();
}

static char *get_network_state_string(enum mdm_hl7800_network_state state)
{
	// clang-format off
    switch (state) {
        PREFIXED_SWITCH_CASE_RETURN_STRING(HL7800, NOT_REGISTERED);
        PREFIXED_SWITCH_CASE_RETURN_STRING(HL7800, HOME_NETWORK);
        PREFIXED_SWITCH_CASE_RETURN_STRING(HL7800, SEARCHING);
        PREFIXED_SWITCH_CASE_RETURN_STRING(HL7800, REGISTRATION_DENIED);
        PREFIXED_SWITCH_CASE_RETURN_STRING(HL7800, OUT_OF_COVERAGE);
        PREFIXED_SWITCH_CASE_RETURN_STRING(HL7800, ROAMING);
        PREFIXED_SWITCH_CASE_RETURN_STRING(HL7800, EMERGENCY);
        PREFIXED_SWITCH_CASE_RETURN_STRING(HL7800, UNABLE_TO_CONFIGURE);
    default:
        return "UNKNOWN";
    }
	// clang-format on
}

static void set_network_state(enum mdm_hl7800_network_state state)
{
	ictx.network_state = state;
	generate_network_state_event();
}

static void generate_network_state_event(void)
{
	struct mdm_hl7800_compound_event event;
	event.code = ictx.network_state;
	event.string = get_network_state_string(ictx.network_state);
	LOG_INF("Network State: %d %s", ictx.network_state, event.string);
	event_handler(HL7800_EVENT_NETWORK_STATE_CHANGE, &event);
}

/* Handler: +CEREG: <n>,<stat>[,[<lac>],[<ci>],[<AcT>]
*  [,[<cause_type>],[<reject_cause>] [,[<Active-Time>],[<Periodic-TAU>]]]] */
static bool on_cmd_network_report_query(struct net_buf **buf, u16_t len)
{
	size_t out_len;
	char value[len + 1];

	out_len = net_buf_linearize(value, sizeof(value) - 1, *buf, 0, len);
	char *pos = strchr(value, ',');
	if (pos) {
		int l = (value + out_len) - pos;
		char val[l];
		strncpy(val, pos + 1, l);
		val[l] = 0;
		set_network_state(strtol(val, NULL, 0));

		/* start work to adjust iface */
		k_delayed_work_cancel(&ictx.iface_status_work);
		k_delayed_work_submit_to_queue(&hl7800_workq,
					       &ictx.iface_status_work,
					       IFACE_WORK_DELAY);
	}

	return true;
}

#ifdef CONFIG_NEWLIB_LIBC
/* Handler: +CCLK: "yy/MM/dd,hh:mm:ss±zz" */
static bool on_cmd_rtc_query(struct net_buf **buf, u16_t len)
{
	struct net_buf *frag = NULL;
	size_t str_len = strlen(TIME_STRING_FORMAT);
	char rtc_string[sizeof(TIME_STRING_FORMAT)];
	memset(rtc_string, 0, sizeof(rtc_string));
	ictx.local_time_valid = false;

	wait_for_modem_data_and_newline(buf, net_buf_frags_len(*buf),
					sizeof(TIME_STRING_FORMAT));

	frag = NULL;
	len = net_buf_findcrlf(*buf, &frag);
	if (!frag) {
		goto done;
	}
	if (len != str_len) {
		LOG_WRN("Unexpected length for RTC string %d (expected:%d)",
			len, str_len);
	} else {
		net_buf_linearize(rtc_string, str_len, *buf, 0, str_len);
		LOG_INF("RTC string: '%s'", log_strdup(rtc_string));
		ictx.local_time_valid = convert_time_string_to_struct(
			&ictx.local_time, &ictx.local_time_offset, rtc_string);
	}
done:
	return true;
}

static bool valid_time_string(const char *time_string)
{
	/* Ensure the all the expected delimiters are present */
	size_t offset = TIME_STRING_DIGIT_STRLEN + TIME_STRING_SEPARATOR_STRLEN;
	size_t i = TIME_STRING_FIRST_SEPARATOR_INDEX;
	for (; i < TIME_STRING_PLUS_MINUS_INDEX; i += offset) {
		if (time_string[i] != TIME_STRING_FORMAT[i]) {
			return false;
		}
	}
	/* The last character is the offset from UTC and can be either
	 * positive or negative.  The last " is also handled here. */
	if ((time_string[i] == '+' || time_string[i] == '-') &&
	    (time_string[i + offset] == '"')) {
		return true;
	}
	return false;
}
#endif

int get_next_time_string_digit(int *failure_cnt, char **pp, int min, int max)
{
	char digits[TIME_STRING_DIGIT_STRLEN + SIZE_OF_NUL];
	memset(digits, 0, sizeof(digits));
	memcpy(digits, *pp, TIME_STRING_DIGIT_STRLEN);
	*pp += TIME_STRING_DIGIT_STRLEN + TIME_STRING_SEPARATOR_STRLEN;
	int result = strtol(digits, NULL, 0);
	if (result > max) {
		*failure_cnt += 1;
		return max;
	} else if (result < min) {
		*failure_cnt += 1;
		return min;
	} else {
		return result;
	}
}

#ifdef CONFIG_NEWLIB_LIBC
static bool convert_time_string_to_struct(struct tm *tm, s32_t *offset,
					  char *time_string)
{
	int fc = 0;
	char *ptr = time_string;
	if (!valid_time_string(ptr)) {
		return false;
	}
	ptr = &ptr[TIME_STRING_FIRST_DIGIT_INDEX];
	tm->tm_year = TIME_STRING_TO_TM_STRUCT_YEAR_OFFSET +
		      get_next_time_string_digit(&fc, &ptr, TM_YEAR_RANGE);
	tm->tm_mon =
		get_next_time_string_digit(&fc, &ptr, TM_MONTH_RANGE_PLUS_1) -
		1;
	tm->tm_mday = get_next_time_string_digit(&fc, &ptr, TM_DAY_RANGE);
	tm->tm_hour = get_next_time_string_digit(&fc, &ptr, TM_HOUR_RANGE);
	tm->tm_min = get_next_time_string_digit(&fc, &ptr, TM_MIN_RANGE);
	tm->tm_sec = get_next_time_string_digit(&fc, &ptr, TM_SEC_RANGE);
	tm->tm_isdst = 0;
	*offset = (s32_t)get_next_time_string_digit(&fc, &ptr,
						    QUARTER_HOUR_RANGE) *
		  SECONDS_PER_QUARTER_HOUR;
	if (time_string[TIME_STRING_PLUS_MINUS_INDEX] == '-') {
		*offset *= -1;
	}
	return (fc == 0);
}
#endif

/* Handler: +CEREG: <stat>[,[<lac>],[<ci>],[<AcT>]
*  [,[<cause_type>],[<reject_cause>] [,[<Active-Time>],[<Periodic-TAU>]]]] */
static bool on_cmd_network_report(struct net_buf **buf, u16_t len)
{
	size_t out_len;

	out_len = net_buf_linearize(ictx.mdm_network_status,
				    sizeof(ictx.mdm_network_status) - 1, *buf,
				    0, len);
	ictx.mdm_network_status[out_len] = 0;
	LOG_DBG("Network status: %s", log_strdup(ictx.mdm_network_status));
	char *pos = strchr(ictx.mdm_network_status, ',');
	if (pos) {
		int l = pos - ictx.mdm_network_status;
		char val[l];
		strncpy(val, ictx.mdm_network_status, l);
		val[l] = 0;
		set_network_state(strtol(val, NULL, 0));
	} else {
		set_network_state(strtol(ictx.mdm_network_status, NULL, 0));
	}

	/* start work to adjust iface */
	k_delayed_work_cancel(&ictx.iface_status_work);
	k_delayed_work_submit_to_queue(&hl7800_workq, &ictx.iface_status_work,
				       IFACE_WORK_DELAY);

	return true;
}

/* Handler: +KCELLMEAS: <RSRP>,<Downlink Path Loss>,<PUSCH Tx Power>,
*                       <PUCCH Tx Power>,<SiNR>
*/
static bool on_cmd_atcmdinfo_rssi(struct net_buf **buf, u16_t len)
{
	/* number of ',' delimiters in this response */
	int numDelims = 4;
	char *delims[numDelims];
	size_t out_len;
	char value[len + 1];
	char *searchStart;

	out_len = net_buf_linearize(value, sizeof(value), *buf, 0, len);
	value[out_len] = 0;
	searchStart = value;

	/* find all delimiters */
	for (int i = 0; i < numDelims; i++) {
		delims[i] = strchr(searchStart, ',');
		if (!delims[i]) {
			LOG_ERR("Could not find delim %d, val: %s", i,
				log_strdup(value));
			goto done;
		}
		/* Start next search after current delim location */
		searchStart = delims[i] + 1;
	}
	/* the first value in the message is the RSRP */
	ictx.mdm_ctx.data_rssi = strtol(value, NULL, 10);
	/* the 4th ',' (last in the msg) is the start of the SINR */
	ictx.mdm_sinr = strtol(delims[3] + 1, NULL, 10);
	if ((delims[1] - delims[0]) == 1) {
		/* there is no value between the first and second
		*  delimiter, signal is unknown
		*/
		LOG_INF("RSSI (RSRP): UNKNOWN");
	} else {
		LOG_INF("RSSI (RSRP): %d SINR: %d", ictx.mdm_ctx.data_rssi,
			ictx.mdm_sinr);
		event_handler(HL7800_EVENT_RSSI, &ictx.mdm_ctx.data_rssi);
		event_handler(HL7800_EVENT_SINR, &ictx.mdm_sinr);
	}
done:
	return true;
}

/* Handle the "OK" response from an AT command or a socket call */
static bool on_cmd_sockok(struct net_buf **buf, u16_t len)
{
	struct hl7800_socket *sock = NULL;

	ictx.last_error = 0;
	sock = socket_from_id(ictx.last_socket_id);
	if (!sock) {
		k_sem_give(&ictx.response_sem);
	} else {
		k_sem_give(&sock->sock_send_sem);
	}
	return true;
}

/* Handler: +KTCP_IND/+KUDP_IND */
static bool on_cmd_sock_ind(struct net_buf **buf, u16_t len)
{
	struct hl7800_socket *sock = NULL;
	char *delim;
	char value[len + 1];
	size_t out_len;
	int id;

	ictx.last_error = 0;

	out_len = net_buf_linearize(value, sizeof(value), *buf, 0, len);
	value[out_len] = 0;

	/* find ',' because this is the format we expect */
	delim = strchr(value, ',');
	if (!delim) {
		LOG_ERR("+K**P_IND could not find ','");
		goto done;
	}

	id = strtol(value, NULL, 10);
	LOG_DBG("+K**P_IND ID: %d", id);
	sock = socket_from_id(id);
	if (sock) {
		k_sem_give(&sock->sock_send_sem);
	} else {
		LOG_ERR("Could not find socket id (%d)", id);
	}

done:
	return true;
}

/* Handler: ERROR */
static bool on_cmd_sockerror(struct net_buf **buf, u16_t len)
{
	if (len > 0) {
		char string[len + 1];
		memset(string, 0, sizeof(string));
		net_buf_linearize(string, sizeof(string), *buf, 0, len);
		LOG_ERR("'%s'", string);
	}

	struct hl7800_socket *sock = NULL;

	ictx.last_error = -EIO;
	sock = socket_from_id(ictx.last_socket_id);
	if (!sock) {
		k_sem_give(&ictx.response_sem);
	} else {
		k_sem_give(&sock->sock_send_sem);
	}

	return true;
}

/* Handler: CME/CMS Error */
static bool on_cmd_sock_error_code(struct net_buf **buf, u16_t len)
{
	struct hl7800_socket *sock = NULL;
	char value[len + 1];
	size_t out_len;

	out_len = net_buf_linearize(value, sizeof(value), *buf, 0, len);
	value[out_len] = 0;

	LOG_ERR("Error code: %s", log_strdup(value));

	ictx.last_error = -EIO;
	sock = socket_from_id(ictx.last_socket_id);
	if (!sock) {
		k_sem_give(&ictx.response_sem);
	} else {
		k_sem_give(&sock->sock_send_sem);
	}

	return true;
}

static void sock_notif_cb_work(struct k_work *work)
{
	struct hl7800_socket *sock = NULL;

	sock = CONTAINER_OF(work, struct hl7800_socket, notif_work);

	if (!sock) {
		LOG_ERR("sock_notif_cb_work: Socket not found");
		return;
	}

	hl7800_lock();
	/* send null packet */
	if (sock->recv_pkt != NULL) {
		/* we are in the middle of RX,
		requeue this and try again */
		k_delayed_work_submit_to_queue(&hl7800_workq, &sock->notif_work,
					       MDM_SOCK_NOTIF_DELAY);
	} else {
		LOG_DBG("Sock %d trigger NULL packet", sock->socket_id);
		k_work_submit_to_queue(&hl7800_workq, &sock->recv_cb_work);
		sock->error = false;
	}
	hl7800_unlock();
}

/* Handler: +KTCP_NOTIF/+KUDP_NOTIF */
static bool on_cmd_sock_notif(struct net_buf **buf, u16_t len)
{
	struct hl7800_socket *sock = NULL;
	char *delim;
	char value[len + 1];
	size_t out_len;
	u8_t notifVal;
	bool err = false;
	bool triggerSem = true;
	int id;

	out_len = net_buf_linearize(value, sizeof(value), *buf, 0, len);
	value[out_len] = 0;

	/* find ',' because this is the format we expect */
	delim = strchr(value, ',');
	if (!delim) {
		LOG_ERR("+K**P_NOTIF could not find ','");
		goto done;
	}

	notifVal = strtol(delim + 1, NULL, 10);
	switch (notifVal) {
	case HL7800_TCP_DATA_SND:
		err = false;
		ictx.last_error = 0;
		break;
	case HL7800_TCP_DISCON:
		triggerSem = false;
		err = true;
		ictx.last_error = -EIO;
		break;
	default:
		err = true;
		ictx.last_error = -EIO;
		break;
	}

	id = strtol(value, NULL, 10);
	LOG_WRN("+K**P_NOTIF: %d,%d", id, notifVal);

	sock = socket_from_id(id);
	if (err) {
		if (sock) {
			/* Send NULL packet to callback to notify upper stack layers
			*  that the peer closed the connection or there was an error.
			*  This is so an app will not get stuck in recv() forever.
			*  Let's do the callback processing in a different work queue
			*  so RX is not delayed.
			*/
			sock->error = true;
			sock->error_val = notifVal;
			k_delayed_work_submit_to_queue(&hl7800_workq,
						       &sock->notif_work,
						       MDM_SOCK_NOTIF_DELAY);
			if (triggerSem) {
				k_sem_give(&sock->sock_send_sem);
			}
		} else {
			LOG_ERR("Could not find socket id (%d)", id);
		}
	}
done:
	return true;
}

/* Handler: +KTCPCFG/+KUDPCFG: <session_id> */
static bool on_cmd_sockcreate(struct net_buf **buf, u16_t len)
{
	size_t out_len;
	char value[len + 1];
	struct hl7800_socket *sock = NULL;

	out_len = net_buf_linearize(value, sizeof(value), *buf, 0, len);
	value[out_len] = 0;
	ictx.last_socket_id = strtol(value, NULL, 10);
	LOG_DBG("+K**PCFG: %d", ictx.last_socket_id);

	/* check if the socket has been created already */
	sock = socket_from_id(ictx.last_socket_id);
	if (!sock) {
		/* look up new socket by special id */
		sock = socket_from_id(MDM_MAX_SOCKETS + 1);
		if (!sock) {
			LOG_ERR("No matching socket");
			goto done;
		}
	}

	sock->socket_id = ictx.last_socket_id;
	sock->created = true;
	/* don't give back semaphore -- OK to follow */
done:
	return true;
}

static void sockreadrecv_cb_work(struct k_work *work)
{
	struct hl7800_socket *sock = NULL;
	struct net_pkt *pkt;

	sock = CONTAINER_OF(work, struct hl7800_socket, recv_cb_work);
	if (!sock) {
		LOG_ERR("Sock not found");
		return;
	}
	LOG_DBG("Sock %d RX CB", sock->socket_id);
	/* return data */
	pkt = sock->recv_pkt;
	sock->recv_pkt = NULL;
	if (sock->recv_cb) {
		sock->recv_cb(sock->context, pkt, NULL, NULL, 0,
			      sock->recv_user_data);
	} else {
		net_pkt_unref(pkt);
	}
}

static void sock_read(struct net_buf **buf, u16_t len)
{
	struct hl7800_socket *sock = NULL;
	struct net_buf *frag;
	u8_t c = 0U;
	int i, hdr_len;
	char okResp[sizeof(OK_STRING)];
	char eof[sizeof(EOF_PATTERN)];
	size_t out_len;

	sock = socket_from_id(ictx.last_socket_id);
	if (!sock) {
		LOG_ERR("Socket not found! (%d)", ictx.last_socket_id);
		goto done;
	}

	if (sock->error) {
		/* cancel notif work and restart */
		k_delayed_work_cancel(&sock->notif_work);
		k_delayed_work_submit_to_queue(&hl7800_workq, &sock->notif_work,
					       MDM_SOCK_NOTIF_DELAY);
	}

	LOG_DBG("Socket %d RX %u bytes", sock->socket_id, sock->rx_size);

	/* remove ending \r\n from last CONNECT */
	if (net_buf_frags_len(*buf) < 2) {
		/* wait for \n to be RXd.  \r was already RXd. */
		wait_for_modem_data(buf, 0, 1);
	}
	net_buf_skipcrlf(buf);
	if (!*buf) {
		wait_for_modem_data(buf, 0, sock->rx_size);
	}

	LOG_DBG("Processing RX, buf len: %d", net_buf_frags_len(*buf));

	/* allocate an RX pkt */
	sock->recv_pkt = net_pkt_rx_alloc_with_buffer(
		net_context_get_iface(sock->context), sock->rx_size,
		sock->family, sock->ip_proto, BUF_ALLOC_TIMEOUT);
	if (!sock->recv_pkt) {
		LOG_ERR("Failed net_pkt_get_reserve_rx!");
		goto done;
	}

	/* set pkt data */
	net_pkt_set_context(sock->recv_pkt, sock->context);

	/* add IP / protocol headers */
	hdr_len = pkt_setup_ip_data(sock->recv_pkt, sock);

	/* receive data */
	for (i = 0; i < sock->rx_size; i++) {
		/* pull data from buf and advance to the next frag if needed */
		c = net_buf_get_u8(buf);
		/* write data to packet */
		if (net_pkt_write_u8(sock->recv_pkt, c)) {
			LOG_ERR("Unable to add data! Aborting! Bytes RXd:%d",
				i);
			goto rx_err;
		}

		if (!*buf && i < sock->rx_size) {
			LOG_DBG("RX more data, bytes RXd:%d", i + 1);
			/* wait for at least one more byte */
			wait_for_modem_data(buf, 0, 1);
			if (!*buf) {
				LOG_ERR("No data in buf!");
				break;
			}
		}
	}

	LOG_DBG("Got all data, get EOF and OK (buf len:%d)",
		net_buf_frags_len(*buf));

	if (!*buf || (net_buf_frags_len(*buf) < strlen(EOF_PATTERN))) {
		wait_for_modem_data(buf, net_buf_frags_len(*buf),
				    strlen(EOF_PATTERN));
		if (!*buf) {
			LOG_ERR("No EOF present");
			goto rx_err;
		}
	}

	out_len = net_buf_linearize(eof, sizeof(eof), *buf, 0,
				    strlen(EOF_PATTERN));
	eof[out_len] = 0;
	/* remove EOF pattern from buffer */
	net_buf_remove(buf, strlen(EOF_PATTERN));
	if (strcmp(eof, EOF_PATTERN)) {
		LOG_ERR("Could not find EOF");
		goto rx_err;
	}

	/* Make sure we have \r\nOK\r\n length in the buffer */
	if (!*buf || (net_buf_frags_len(*buf) < strlen(OK_STRING) + 4)) {
		wait_for_modem_data(buf, net_buf_frags_len(*buf),
				    strlen(OK_STRING) + 4);
		if (!*buf) {
			LOG_ERR("No OK present");
			goto rx_err;
		}
	}

	frag = NULL;
	len = net_buf_findcrlf(*buf, &frag);
	if (!frag) {
		LOG_ERR("Unable to find OK start");
		goto rx_err;
	}
	/* remove \r\n before OK */
	net_buf_skipcrlf(buf);

	out_len = net_buf_linearize(okResp, sizeof(okResp), *buf, 0,
				    strlen(OK_STRING));
	okResp[out_len] = 0;
	/* remove the message from the buffer */
	net_buf_remove(buf, strlen(OK_STRING));
	if (strcmp(okResp, OK_STRING)) {
		LOG_ERR("Could not find OK");
		goto rx_err;
	}

	/* remove \r\n after OK */
	net_buf_skipcrlf(buf);

	net_pkt_cursor_init(sock->recv_pkt);
	net_pkt_set_overwrite(sock->recv_pkt, true);

	if (hdr_len > 0) {
		net_pkt_skip(sock->recv_pkt, hdr_len);
	}

	/* Let's do the callback processing in a different work queue in
	* case the app takes a long time.
	*/
	k_work_submit_to_queue(&hl7800_workq, &sock->recv_cb_work);
	LOG_DBG("Sock %d RX done", sock->socket_id);
	goto done;
rx_err:
	net_pkt_unref(sock->recv_pkt);
	sock->recv_pkt = NULL;
done:
	sock->state = SOCK_IDLE;
	hl7800_TX_unlock();
}

static bool on_cmd_connect(struct net_buf **buf, u16_t len)
{
	bool removeDataFromBuffer = true;
	struct hl7800_socket *sock = NULL;

	sock = socket_from_id(ictx.last_socket_id);
	if (!sock) {
		LOG_ERR("Sock (%d) not found", ictx.last_socket_id);
		goto done;
	}

	if (sock->state == SOCK_RX) {
		removeDataFromBuffer = false;
		sock_read(buf, len);
	} else {
		k_sem_give(&sock->sock_send_sem);
	}

done:
	return removeDataFromBuffer;
}

static int start_socket_rx(struct hl7800_socket *sock, u16_t rxSize)
{
	char sendbuf[sizeof("AT+KTCPRCV=##,####")];

	if ((sock->socket_id <= 0) || (sock->rx_size <= 0)) {
		LOG_WRN("Cannot start socket RX, ID: %d rx size: %d",
			sock->socket_id, sock->rx_size);
		return -1;
	}

	LOG_DBG("Start socket RX ID:%d size:%d", sock->socket_id, rxSize);
	sock->state = SOCK_RX;
	if (sock->type == SOCK_DGRAM) {
#if defined(CONFIG_NET_IPV4)
		if (rxSize > (net_if_get_mtu(ictx.iface) - NET_IPV4UDPH_LEN)) {
			sock->rx_size =
				net_if_get_mtu(ictx.iface) - NET_IPV4UDPH_LEN;
		}
#else
#error IPV6 not supported in HL7800 driver
#endif
		snprintk(sendbuf, sizeof(sendbuf), "AT+KUDPRCV=%d,%u",
			 sock->socket_id, rxSize);
	} else {
#if defined(CONFIG_NET_IPV4)
		if (rxSize > (net_if_get_mtu(ictx.iface) - NET_IPV4TCPH_LEN)) {
			sock->rx_size =
				net_if_get_mtu(ictx.iface) - NET_IPV4TCPH_LEN;
		}
#else
#error IPV6 not supported in HL7800 driver
#endif
		snprintk(sendbuf, sizeof(sendbuf), "AT+KTCPRCV=%d,%u",
			 sock->socket_id, sock->rx_size);
	}

	/* Send AT+K**PRCV, The modem
	* will respond with "CONNECT" and the data requested
	* and then "OK" or "ERROR".
	* The rest of the data processing will be handled
	* once CONNECT is RXd.
	*/
	send_at_cmd(sock, sendbuf, K_NO_WAIT, 0, false);
	return 0;
}

static void sock_rx_data_cb_work(struct k_work *work)
{
	struct hl7800_socket *sock = NULL;
	int rc;

	sock = CONTAINER_OF(work, struct hl7800_socket, rx_data_work);

	if (!sock) {
		LOG_ERR("sock_rx_data_cb_work: Socket not found");
		return;
	}

	hl7800_lock();

	/* start RX */
	rc = start_socket_rx(sock, sock->rx_size);

	/* Only unlock the RX because we just locked it above.
	*  At the end of socket RX, the TX will be unlocked.
	*/
	hl7800_RX_unlock();
	if (rc < 0) {
		/* we didnt start socket RX so unlock TX now. */
		hl7800_TX_unlock();
	}
}

/* Handler: +KTCP_DATA/+KUDP_DATA: <socket_id>,<left_bytes> */
static bool on_cmd_sockdataind(struct net_buf **buf, u16_t len)
{
	int socket_id, left_bytes, rc;
	size_t out_len;
	char *delim;
	char value[sizeof("##,####")];
	struct hl7800_socket *sock = NULL;
	bool unlock = false;
	bool deferRx = false;

	if (!hl7800_TX_locked()) {
		hl7800_TX_lock();
		unlock = true;
	} else {
		deferRx = true;
	}

	out_len = net_buf_linearize(value, sizeof(value) - 1, *buf, 0, len);
	value[out_len] = 0;

	/* First comma separator marks the end of socket_id */
	delim = strchr(value, ',');
	if (!delim) {
		LOG_ERR("Missing comma");
		goto error;
	}

	/* replace comma with null */
	*delim++ = '\0';
	socket_id = strtol(value, NULL, 0);

	/* second param is for left_bytes */
	left_bytes = strtol(delim, NULL, 0);

	sock = socket_from_id(socket_id);
	if (!sock) {
		LOG_ERR("Unable to find socket_id:%d", socket_id);
		goto error;
	}

	sock->rx_size = left_bytes;
	if (deferRx) {
		LOG_DBG("Defer socket RX -> ID: %d bytes: %u", socket_id,
			left_bytes);
		k_work_submit_to_queue(&hl7800_workq, &sock->rx_data_work);
	} else {
		if (left_bytes > 0) {
			rc = start_socket_rx(sock, left_bytes);
			if (rc < 0) {
				goto error;
			}
			goto done;
		}
	}
error:
	if (unlock) {
		hl7800_TX_unlock();
	}
done:
	return true;
}

static inline struct net_buf *read_rx_allocator(s32_t timeout, void *user_data)
{
	return net_buf_alloc((struct net_buf_pool *)user_data, timeout);
}

static size_t hl7800_read_rx(struct net_buf **buf)
{
	u8_t uart_buffer[MDM_RECV_BUF_SIZE];
	size_t bytes_read, totalRead;
	int ret;
	u16_t rx_len;
	bytes_read = 0, totalRead = 0;

	/* read all of the data from mdm_receiver */
	while (true) {
		ret = mdm_receiver_recv(&ictx.mdm_ctx, uart_buffer,
					sizeof(uart_buffer), &bytes_read);
		if (ret < 0 || bytes_read == 0) {
			/* mdm_receiver buffer is empty */
			break;
		}

#if defined(HL7800_ENABLE_VERBOSE_MODEM_RECV_HEXDUMP)
		LOG_HEXDUMP_DBG((const u8_t *)&uart_buffer, bytes_read,
				"HL7800 RX");
#endif
		/* make sure we have storage */
		if (!*buf) {
			*buf = net_buf_alloc(&mdm_recv_pool, BUF_ALLOC_TIMEOUT);
			if (!*buf) {
				LOG_ERR("Can't allocate RX data! "
					"Skipping data!");
				break;
			}
		}

		rx_len =
			net_buf_append_bytes(*buf, bytes_read, uart_buffer,
					     BUF_ALLOC_TIMEOUT,
					     read_rx_allocator, &mdm_recv_pool);
		if (rx_len < bytes_read) {
			LOG_ERR("Data was lost! read %u of %u!", rx_len,
				bytes_read);
		}
		totalRead += bytes_read;
	}

	return totalRead;
}

/* RX thread */
static void hl7800_rx(void)
{
	struct net_buf *rx_buf = NULL;
	struct net_buf *frag = NULL;
	int i, cmpRes;
	u16_t len;
	size_t out_len;
	bool cmd_handled = false;
	static char rx_msg[MDM_HANDLER_MATCH_MAX_LEN];
	bool unlock = false;
	bool removeLineFromBuf = true;

	static const struct cmd_handler handlers[] = {
		/* MODEM Information */
		CMD_HANDLER("AT+CGMI", atcmdinfo_manufacturer),
		CMD_HANDLER("AT+CGMM", atcmdinfo_model),
		CMD_HANDLER("AT+CGMR", atcmdinfo_revision),
		CMD_HANDLER("AT+CGSN", atcmdinfo_imei),
		CMD_HANDLER("AT+KGSN=3", atcmdinfo_serial_number),
		CMD_HANDLER("+KCELLMEAS: ", atcmdinfo_rssi),
		CMD_HANDLER("+CGCONTRDP: ", atcmdinfo_ipaddr),
		CMD_HANDLER("+COPS: ", atcmdinfo_operator_status),
		CMD_HANDLER("+KSRAT: ", radio_tech_status),
		CMD_HANDLER("+KBNDCFG: ", radio_band_configuration),
		CMD_HANDLER("+CCID: ", atcmdinfo_iccid),
		CMD_HANDLER("ACTIVE PROFILE:", atcmdinfo_active_profile),
		CMD_HANDLER("STORED PROFILE 0:", atcmdinfo_stored_profile0),
		CMD_HANDLER("STORED PROFILE 1:", atcmdinfo_stored_profile1),
		CMD_HANDLER("+WPPP: 1,1,", atcmdinfo_pdp_authentication_cfg),
		CMD_HANDLER("+CGDCONT: 1", atcmdinfo_pdp_context),
		CMD_HANDLER("AT+CEREG?", network_report_query),
#ifdef CONFIG_NEWLIB_LIBC
		CMD_HANDLER("+CCLK: ", rtc_query),
#endif

		/* UNSOLICITED modem information */
		/* mobile startup report */
		CMD_HANDLER("+KSUP: ", startup_report),
		/* network status */
		CMD_HANDLER("+CEREG: ", network_report),

		/* SOLICITED CMD AND SOCKET RESPONSES */
		CMD_HANDLER("OK", sockok),
		CMD_HANDLER("ERROR", sockerror),

		/* SOLICITED SOCKET RESPONSES */
		CMD_HANDLER("+CME ERROR: ", sock_error_code),
		CMD_HANDLER("+CMS ERROR: ", sock_error_code),
		CMD_HANDLER("+CEER: ", sockerror),
		CMD_HANDLER("+KTCPCFG: ", sockcreate),
		CMD_HANDLER("+KUDPCFG: ", sockcreate),
		CMD_HANDLER(CONNECT_STRING, connect),
		CMD_HANDLER("NO CARRIER", sockerror),

		/* UNSOLICITED SOCKET RESPONSES */
		CMD_HANDLER("+KTCP_IND: ", sock_ind),
		CMD_HANDLER("+KUDP_IND: ", sock_ind),
		CMD_HANDLER("+KTCP_NOTIF: ", sock_notif),
		CMD_HANDLER("+KUDP_NOTIF: ", sock_notif),
		CMD_HANDLER("+KTCP_DATA: ", sockdataind),
		CMD_HANDLER("+KUDP_DATA: ", sockdataind),
	};

	while (true) {
		/* wait for incoming data */
		k_sem_take(&ictx.mdm_ctx.rx_sem, K_FOREVER);

		hl7800_read_rx(&rx_buf);
		/* If an external module hasn't locked the command processor, then do so now. */
		if (!hl7800_RX_locked()) {
			hl7800_RX_lock();
			unlock = true;
		} else {
			unlock = false;
		}

		while (rx_buf) {
			removeLineFromBuf = true;
			cmd_handled = false;
			net_buf_skipcrlf(&rx_buf);
			if (!rx_buf) {
				break;
			}

			frag = NULL;
			len = net_buf_findcrlf(rx_buf, &frag);
			if (!frag) {
				break;
			}

			out_len = net_buf_linearize(rx_msg, sizeof(rx_msg),
						    rx_buf, 0, len);

			/* look for matching data handlers */
			i = -1;
			for (i = 0; i < ARRAY_SIZE(handlers); i++) {
				if (ictx.search_no_id_resp) {
					cmpRes = strncmp(ictx.no_id_resp_cmd,
							 handlers[i].cmd,
							 handlers[i].cmd_len);
				} else {
					cmpRes =
						strncmp(rx_msg, handlers[i].cmd,
							handlers[i].cmd_len);
				}

				if (cmpRes == 0) {
					/* found a matching handler */

					/* skip cmd_len */
					if (!ictx.search_no_id_resp) {
						rx_buf = net_buf_skip(
							rx_buf,
							handlers[i].cmd_len);
					}

					/* locate next cr/lf */
					frag = NULL;
					len = net_buf_findcrlf(rx_buf, &frag);
					if (!frag) {
						break;
					}

					LOG_DBG("HANDLE %s (len:%u)",
						handlers[i].cmd, len);
					/* call handler */
					if (handlers[i].func) {
						removeLineFromBuf =
							handlers[i].func(
								&rx_buf, len);
					}
					cmd_handled = true;
					ictx.search_no_id_resp = false;
					frag = NULL;
					/* make sure buf still has data */
					if (!rx_buf) {
						break;
					}

					/*
					* We've handled the current line
					* and need to exit the "search for
					* handler loop".  Let's skip any
					* "extra" data and look for the next
					* CR/LF, leaving us ready for the
					* next handler search.
					*/
					len = net_buf_findcrlf(rx_buf, &frag);
					break;
				}
			}

#ifdef HL7800_LOG_UNHANDLED_RX_MSGS
			/* Handle unhandled commands */
			if (!cmd_handled && frag && len > 1) {
				char msg[len + 1];
				out_len = net_buf_linearize(msg, sizeof(msg),
							    rx_buf, 0, len);
				msg[out_len] = 0;
				LOG_HEXDUMP_DBG((const u8_t *)&msg, len,
						"UNHANDLED RX");
			}
#endif
			if (removeLineFromBuf && frag && rx_buf) {
				/* clear out processed line (buffers) */
				net_buf_remove(&rx_buf, len);
			}
		}

		if (unlock) {
			hl7800_RX_unlock();
		}

		/* give up time if we have a solid stream of data */
		k_yield();
	}
}

static void modem_disconnect_uart_tx(void)
{
	int ret = gpio_pin_configure(ictx.gpio_port_dev[MDM_UART_TX],
				     pinconfig[MDM_UART_TX].pin,
				     pinconfig[MDM_UART_TX].sleep_config);
	if (ret) {
		LOG_ERR("Error disconnecting UART TX (%d)", ret);
	}
}

static void modem_connect_uart_tx(void)
{
	int ret = gpio_pin_configure(ictx.gpio_port_dev[MDM_UART_TX],
				     pinconfig[MDM_UART_TX].pin,
				     pinconfig[MDM_UART_TX].config);
	if (ret) {
		LOG_ERR("Error connecting UART TX (%d)", ret);
	}
}

static void modem_disconnect_uart_rts(void)
{
	int ret = gpio_pin_configure(ictx.gpio_port_dev[MDM_UART_RTS],
				     pinconfig[MDM_UART_RTS].pin,
				     pinconfig[MDM_UART_RTS].sleep_config);
	if (ret) {
		LOG_ERR("Error disconnecting UART RTS (%d)", ret);
	}
}

static void modem_connect_uart_rts(void)
{
	int ret = gpio_pin_configure(ictx.gpio_port_dev[MDM_UART_RTS],
				     pinconfig[MDM_UART_RTS].pin,
				     pinconfig[MDM_UART_RTS].config);
	if (ret) {
		LOG_ERR("Error connecting UART RTS (%d)", ret);
	}
}

static void modem_disconnect_uart_rx(void)
{
	int rc = gpio_pin_configure(ictx.gpio_port_dev[MDM_UART_RX],
				    pinconfig[MDM_UART_RX].pin,
				    pinconfig[MDM_UART_RX].sleep_config);
	if (rc) {
		LOG_ERR("Error disconnecting UART RX (%d)", rc);
	}
}

static void modem_connect_uart_rx(void)
{
	int rc = gpio_pin_configure(ictx.gpio_port_dev[MDM_UART_RX],
				    pinconfig[MDM_UART_RX].pin,
				    pinconfig[MDM_UART_RX].config);
	if (rc) {
		LOG_ERR("Error connecting UART RX (%d)", rc);
	}
}

static void modem_disconnect_uart_cts(void)
{
	int rc = gpio_pin_configure(ictx.gpio_port_dev[MDM_UART_CTS],
				    pinconfig[MDM_UART_CTS].pin,
				    pinconfig[MDM_UART_CTS].sleep_config);
	if (rc) {
		LOG_ERR("Error disconnecting UART CTS (%d)", rc);
	}
}

static void modem_connect_uart_cts(void)
{
	int rc = gpio_pin_configure(ictx.gpio_port_dev[MDM_UART_CTS],
				    pinconfig[MDM_UART_CTS].pin,
				    pinconfig[MDM_UART_CTS].config);
	if (rc) {
		LOG_ERR("Error connecting UART CTS (%d)", rc);
	}
}

static void shutdown_uart(void)
{
#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
	Z_LOG(HL7800_IO_LOG_LEVEL, "Power OFF the UART");
	int rc = device_set_power_state(ictx.mdm_ctx.uart_dev,
					DEVICE_PM_OFF_STATE, NULL, NULL);
	if (rc) {
		LOG_ERR("Error disabling UART peripheral (%d)", rc);
	}
	modem_disconnect_uart_tx();
	modem_disconnect_uart_rts();
	modem_disconnect_uart_rx();
	modem_disconnect_uart_cts();
#endif
}

static void power_on_uart(void)
{
#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
	Z_LOG(HL7800_IO_LOG_LEVEL, "Power ON the UART");
	modem_connect_uart_tx();
	modem_connect_uart_rts();
	modem_connect_uart_rx();
	modem_connect_uart_cts();
	int rc = device_set_power_state(ictx.mdm_ctx.uart_dev,
					DEVICE_PM_ACTIVE_STATE, NULL, NULL);
	if (rc) {
		LOG_ERR("Error enabling UART peripheral (%d)", rc);
	}
#endif
}

/* Make sure all IO voltages are removed for proper reset. */
static void prepare_io_for_reset(void)
{
	Z_LOG(HL7800_IO_LOG_LEVEL, "Preparing IO for reset/sleep");

	ictx.wait_for_KSUP = true;
	ictx.wait_for_KSUP_tries = 0;
	shutdown_uart();
	modem_assert_wake(false);
	modem_assert_pwr_on(false);
	modem_assert_fast_shutd(false);
	modem_assert_uart_dtr(false);
}

static void mdm_vgpio_work_cb(struct k_work *item)
{
	ARG_UNUSED(item);

	hl7800_lock();
	if (!ictx.vgpio_state) {
		if (ictx.sleep_state != HL7800_SLEEP_STATE_ASLEEP) {
			set_sleep_state(HL7800_SLEEP_STATE_ASLEEP);
		}
		if (ictx.iface && ictx.initialized &&
		    net_if_is_up(ictx.iface)) {
			net_if_down(ictx.iface);
		}
	}
	hl7800_unlock();
}

void mdm_vgpio_callback_isr(struct device *port, struct gpio_callback *cb,
			    u32_t pins)
{
	gpio_pin_read(ictx.gpio_port_dev[MDM_VGPIO], pinconfig[MDM_VGPIO].pin,
		      &ictx.vgpio_state);
	Z_LOG(HL7800_IO_LOG_LEVEL, "VGPIO:%d", ictx.vgpio_state);

	if (!ictx.vgpio_state) {
		prepare_io_for_reset();
		if (!ictx.restarting && ictx.initialized) {
			ictx.reconfig_GPRS = true;
		}
		check_hl7800_awake();
	} else {
		/* The peripheral must be enabled in ISR context because the driver may be
		 * waiting for +KSUP or waiting to send commands.
		 * This can occur, for example, during a modem reset.
	 	 */
		power_on_uart();
		allow_sleep(false);
	}

	/* When the network state changes a semaphore must be taken.
	 * This can't be done in interrupt context because the wait time != 0.
	 */
	k_work_submit_to_queue(&hl7800_workq, &ictx.mdm_vgpio_work);
}

void mdm_uart_dsr_callback_isr(struct device *port, struct gpio_callback *cb,
			       u32_t pins)
{
	gpio_pin_read(ictx.gpio_port_dev[MDM_UART_DSR],
		      pinconfig[MDM_UART_DSR].pin, &ictx.dsr_state);

	Z_LOG(HL7800_IO_LOG_LEVEL, "MDM_UART_DSR:%d", ictx.dsr_state);
}

void mdm_gpio6_callback_isr(struct device *port, struct gpio_callback *cb,
			    u32_t pins)
{
	gpio_pin_read(ictx.gpio_port_dev[MDM_GPIO6], pinconfig[MDM_GPIO6].pin,
		      &ictx.gpio6_state);
	check_hl7800_awake();

	Z_LOG(HL7800_IO_LOG_LEVEL, "MDM_GPIO6:%d", ictx.gpio6_state);
}

void mdm_uart_cts_callback(struct device *port, struct gpio_callback *cb,
			   u32_t pins)
{
	gpio_pin_read(ictx.gpio_port_dev[MDM_UART_CTS],
		      pinconfig[MDM_UART_CTS].pin, &ictx.cts_state);
	/* CTS toggles A LOT, comment out the debug print unless we really need it. */
	/* 	Z_LOG(HL7800_IO_LOG_LEVEL, "MDM_UART_CTS:%d", val); */

	check_hl7800_awake();
}

static void modem_reset(void)
{
	LOG_INF("Modem Reset");

	prepare_io_for_reset();

	/* Hard reset the modem (>20 milliseconds required) */
	gpio_pin_write(ictx.gpio_port_dev[MDM_RESET], pinconfig[MDM_RESET].pin,
		       MDM_RESET_ASSERTED);
	k_sleep(MDM_RESET_LOW_TIME);

	ictx.mdm_startup_reporting_on = false;
	set_sleep_state(HL7800_SLEEP_STATE_UNINITIALIZED);
	check_hl7800_awake();
	set_network_state(HL7800_NOT_REGISTERED);
	set_startup_state(HL7800_STARTUP_STATE_UNKNOWN);
	k_sem_reset(&ictx.mdm_awake);
}

static void modem_run(void)
{
	gpio_pin_write(ictx.gpio_port_dev[MDM_RESET], pinconfig[MDM_RESET].pin,
		       MDM_RESET_NOT_ASSERTED);
	allow_sleep(false);
	k_sleep(MDM_RESET_HIGH_TIME);
	LOG_INF("Modem Run");
}

static int modem_boot_handler(char *reason)
{
	LOG_DBG("%s", reason);
	int ret = k_sem_take(&ictx.mdm_awake, MDM_BOOT_TIME);
	if (ret) {
		LOG_ERR("Err waiting for boot: %d, DSR: %u", ret,
			ictx.dsr_state);
		return -1;
	} else if (ictx.mdm_startup_state != HL7800_STARTUP_STATE_READY) {
		return -1;
	} else {
		LOG_INF("Modem booted!");
	}

	/* Turn OFF EPS network registration status reporting because
	 * it isn't needed until after initialization is complete.
	 */
	SEND_AT_CMD_EXPECT_OK("AT+CEREG=0");

	/* Determine if echo is on/off by reading the profile */
	/* note: It wasn't clear how to read the active profile so all 3 are read. */
	ictx.mdm_echo_is_on = true;
	SEND_AT_CMD_EXPECT_OK("AT&V");

	if (ictx.mdm_echo_is_on) {
		/* Turn OFF echo (after boot/reset) because a profile hasn't been saved yet */
		SEND_AT_CMD_EXPECT_OK("ATE0");

		/* Save profile 0 */
		SEND_AT_CMD_EXPECT_OK("AT&W");

		/* Reread profiles so echo state can be checked again. */
		SEND_AT_CMD_EXPECT_OK("AT&V");
	}

	__ASSERT(!ictx.mdm_echo_is_on, "Echo should be off");

	/* The Laird bootloader puts the modem into airplane mode ("AT+CFUN=4,0").
	 * The radio is enabled here because airplane mode
	 * survives reset and power removal.
	 */
	SEND_AT_CMD_EXPECT_OK("AT+CFUN=1,0");

	return 0;

error:
	return ret;
}

static int modem_reset_and_configure(void)
{
	int ret = 0;
#ifdef CONFIG_MODEM_HL7800_EDRX
	int edrxActType;
	char setEdrxMsg[sizeof("AT+CEDRXS=2,4,\"0000\"")];
#endif

	ictx.restarting = true;
	if (ictx.iface && net_if_is_up(ictx.iface)) {
		net_if_down(ictx.iface);
	}

	hl7800_stop_rssi_work();

reboot:
	modem_reset();
	modem_run();
	ret = modem_boot_handler("Initialization");
	if (!ictx.mdm_startup_reporting_on) {
		/* Turn on mobile start-up reporting for next reset.
		 * It will indicate if SIM is present.
		 * Its value is saved in non-volatile memory on the HL7800.
		 */
		SEND_AT_CMD_EXPECT_OK("AT+KSREP=1");
		goto reboot;
	} else if (ret < 0) {
		goto error;
	}

	/* turn on numeric error codes */
	SEND_AT_CMD_EXPECT_OK("AT+CMEE=1");

	/* query SIM ICCID */
	SEND_AT_CMD_EXPECT_OK("AT+CCID?");

	/* Query current Radio Access Technology (RAT) */
	SEND_AT_CMD_EXPECT_OK("AT+KSRAT?");

	/* Configure LTE bands */
#if CONFIG_MODEM_HL7800_CONFIGURE_BANDS
	u16_t bands_top = 0;
	u32_t bands_middle = 0, bands_bottom = 0;

#if CONFIG_MODEM_HL7800_BAND_1
	bands_bottom |= 1 << 0;
#endif
#if CONFIG_MODEM_HL7800_BAND_2
	bands_bottom |= 1 << 1;
#endif
#if CONFIG_MODEM_HL7800_BAND_3
	bands_bottom |= 1 << 2;
#endif
#if CONFIG_MODEM_HL7800_BAND_4
	bands_bottom |= 1 << 3;
#endif
#if CONFIG_MODEM_HL7800_BAND_5
	bands_bottom |= 1 << 4;
#endif
#if CONFIG_MODEM_HL7800_BAND_8
	bands_bottom |= 1 << 7;
#endif
#if CONFIG_MODEM_HL7800_BAND_9
	bands_bottom |= 1 << 8;
#endif
#if CONFIG_MODEM_HL7800_BAND_10
	bands_bottom |= 1 << 9;
#endif
#if CONFIG_MODEM_HL7800_BAND_12
	bands_bottom |= 1 << 11;
#endif
#if CONFIG_MODEM_HL7800_BAND_13
	bands_bottom |= 1 << 12;
#endif
#if CONFIG_MODEM_HL7800_BAND_14
	bands_bottom |= 1 << 13;
#endif
#if CONFIG_MODEM_HL7800_BAND_17
	bands_bottom |= 1 << 16;
#endif
#if CONFIG_MODEM_HL7800_BAND_18
	bands_bottom |= 1 << 17;
#endif
#if CONFIG_MODEM_HL7800_BAND_19
	bands_bottom |= 1 << 18;
#endif
#if CONFIG_MODEM_HL7800_BAND_20
	bands_bottom |= 1 << 19;
#endif
#if CONFIG_MODEM_HL7800_BAND_25
	bands_bottom |= 1 << 24;
#endif
#if CONFIG_MODEM_HL7800_BAND_26
	bands_bottom |= 1 << 25;
#endif
#if CONFIG_MODEM_HL7800_BAND_27
	bands_bottom |= 1 << 26;
#endif
#if CONFIG_MODEM_HL7800_BAND_28
	bands_bottom |= 1 << 27;
#endif
#if CONFIG_MODEM_HL7800_BAND_66
	bands_top |= 1 << 1;
#endif

	SEND_AT_CMD_EXPECT_OK("AT+KBNDCFG?");

	/* Check if bands are configured correctly */
	if (ictx.mdm_bands_top != bands_top ||
	    ictx.mdm_bands_middle != bands_middle ||
	    ictx.mdm_bands_bottom != bands_bottom) {
		if (ictx.mdm_bands_top != bands_top) {
			LOG_INF("Top band mismatch, want %04x got %04x",
				bands_top, ictx.mdm_bands_top);
		}
		if (ictx.mdm_bands_middle != bands_middle) {
			LOG_INF("Middle band mismatch, want %08x got %08x",
				bands_middle, ictx.mdm_bands_middle);
		}
		if (ictx.mdm_bands_bottom != bands_bottom) {
			LOG_INF("Bottom band mismatch, want %08x got %08x",
				bands_bottom, ictx.mdm_bands_bottom);
		}

		char newBands[sizeof("AT+KBNDCFG=#,####################")];
		snprintk(newBands, sizeof(newBands),
			 "AT+KBNDCFG=%d,%04x%08x%08x", ictx.mdm_rat, bands_top,
			 bands_middle, bands_bottom);
		SEND_AT_CMD_EXPECT_OK(newBands);

		SEND_AT_CMD_EXPECT_OK("AT+CFUN=1,1");

		modem_boot_handler("LTE bands were just set");
		if (ret < 0) {
			goto error;
		}
	}
#endif

#ifdef CONFIG_MODEM_HL7800_LOW_POWER_MODE

	/* enable GPIO6 low power monitoring */
	SEND_AT_CMD_EXPECT_OK("AT+KHWIOCFG=3,1,6");

	/* Turn on sleep mode */
	SEND_AT_CMD_EXPECT_OK("AT+KSLEEP=0,2,10");

#if CONFIG_MODEM_HL7800_PSM
	/* Turn off eDRX */
	SEND_AT_CMD_EXPECT_OK("AT+CEDRXS=0");

	const char TURN_ON_PSM[] =
		"AT+CPSMS=1,,,\"" CONFIG_MODEM_HL7800_PSM_PERIODIC_TAU
		"\",\"" CONFIG_MODEM_HL7800_PSM_ACTIVE_TIME "\"";
	SEND_AT_CMD_EXPECT_OK(TURN_ON_PSM);
#elif CONFIG_MODEM_HL7800_EDRX
	/* Turn off PSM */
	SEND_AT_CMD_EXPECT_OK("AT+CPSMS=0");

	/* turn on eDRX */
	if (ictx.mdm_rat == MDM_RAT_CAT_NB1) {
		edrxActType = 5;
	} else {
		edrxActType = 4;
	}
	snprintk(setEdrxMsg, sizeof(setEdrxMsg), "AT+CEDRXS=1,%d,\"%s\"",
		 edrxActType, CONFIG_MODEM_HL7800_EDRX_VALUE);
	SEND_AT_CMD_EXPECT_OK(setEdrxMsg);
#endif
#else
	/* Turn off sleep mode */
	SEND_AT_CMD_EXPECT_OK("AT+KSLEEP=2");

	/* Turn off PSM */
	SEND_AT_CMD_EXPECT_OK("AT+CPSMS=0");

	/* Turn off eDRX */
	SEND_AT_CMD_EXPECT_OK("AT+CEDRXS=0");
#endif

	/* modem manufacturer */
	SEND_COMPLEX_AT_CMD("AT+CGMI");

	/* modem model */
	SEND_COMPLEX_AT_CMD("AT+CGMM");

	/* modem revision */
	SEND_COMPLEX_AT_CMD("AT+CGMR");

	/* query modem IMEI */
	SEND_COMPLEX_AT_CMD("AT+CGSN");

	/* query modem serial number */
	SEND_COMPLEX_AT_CMD("AT+KGSN=3");

	/* query SIM ICCID */
	SEND_AT_CMD_EXPECT_OK("AT+CCID?");

	/* An empty string is used here so that it doesn't conflict
	 * with the APN used in the +CGDCONT command.
	 */
	SEND_AT_CMD_EXPECT_OK(SETUP_GPRS_CONNECTION_CMD);

	/* Query PDP context to get APN */
	SEND_AT_CMD_EXPECT_OK("AT+CGDCONT?");

	/* Query PDP authentication context to get APN username/password */
	/* Temporary Workaroud - Ignore error
	 * On some modules this is returning an error and the response data.
	 */
	SEND_AT_CMD_IGNORE_ERROR("AT+WPPP?");

	/* query the network status in case we already registered */
	SEND_COMPLEX_AT_CMD("AT+CEREG?");

	/* Turn on EPS network registration status reporting */
	SEND_AT_CMD_EXPECT_OK("AT+CEREG=4");

	/* The modem has been initialized and now the network interface can be
	 * started in the CEREG message handler.
	 */
	LOG_INF("Modem ready!");
	ictx.restarting = false;
	/* trigger APN update event */
	event_handler(HL7800_EVENT_APN_UPDATE, &ictx.mdm_apn);
	return 0;

error:
	LOG_ERR("Unable to configure modem");
	set_network_state(HL7800_UNABLE_TO_CONFIGURE);
	modem_reset();
	/* Kernel will fault with non-zero return value.
	 * Allow other parts of application to run when modem cannot be configured.
	 */
	return 0;
}

static void mdm_reset_work_callback(struct k_work *item)
{
	ARG_UNUSED(item);

	mdm_hl7800_reset();
}

s32_t mdm_hl7800_reset(void)
{
	int ret;

	hl7800_lock();

	ret = modem_reset_and_configure();

	hl7800_unlock();

	return ret;
}

static int hl7800_power_off(void)
{
	int ret = 0;
	LOG_INF("Powering off modem");
	wakeup_hl7800();
	hl7800_stop_rssi_work();

	/* use the restarting flag to prevent +CEREG updates */
	ictx.restarting = true;

	ret = send_at_cmd(NULL, "AT+CPOF", MDM_CMD_SEND_TIMEOUT, 1, false);
	if (ret) {
		LOG_ERR("AT+CPOF ret:%d", ret);
		return ret;
	}
	/* bring the iface down */
	if (ictx.iface && net_if_is_up(ictx.iface)) {
		net_if_down(ictx.iface);
	}
	LOG_INF("Modem powered off");
	return ret;
}

s32_t mdm_hl7800_power_off(void)
{
	int rc;

	hl7800_lock();
	rc = hl7800_power_off();
	hl7800_unlock();

	return rc;
}

void mdm_hl7800_register_event_callback(
	void (*callback)(enum mdm_hl7800_event event, void *event_data))
{
	int key = irq_lock();
	ictx.event_callback = callback;
	irq_unlock(key);
}

/*** OFFLOAD FUNCTIONS ***/

static bool is_network_ready(void)
{
	return (ictx.sleep_state == HL7800_SLEEP_STATE_AWAKE &&
		net_if_is_up(ictx.iface));
}

static int offload_get(sa_family_t family, enum net_sock_type type,
		       enum net_ip_protocol ip_proto,
		       struct net_context **context)
{
	int ret = 0;
	struct hl7800_socket *sock = NULL;

	if (!is_network_ready()) {
		return -EBUSY;
	}

	hl7800_lock();
	/* new socket */
	sock = socket_get();
	if (!sock) {
		ret = -ENOMEM;
		goto done;
	}

	(*context)->offload_context = sock;
	/* set the context iface index to our iface */
	(*context)->iface = net_if_get_by_iface(ictx.iface);
	sock->family = family;
	sock->type = type;
	sock->ip_proto = ip_proto;
	sock->context = *context;
	sock->socket_id = MDM_MAX_SOCKETS + 1; /* socket # needs assigning */

	/* If UDP, create UDP socket now.
	*  TCP socket needs to be created later once the connection IP address is known.
	*/
	if (type == SOCK_DGRAM) {
		/* check if we need to re-setup GPRS connection */
		if (ictx.reconfig_GPRS) {
			ictx.reconfig_GPRS = false;
			ret = send_at_cmd(sock, SETUP_GPRS_CONNECTION_CMD,
					  MDM_CMD_SEND_TIMEOUT, 0, false);
			if (ret < 0) {
				LOG_ERR("AT+KCNXCFG= ret:%d", ret);
				socket_put(sock);
				goto done;
			}
		}

		ret = send_at_cmd(sock, "AT+KUDPCFG=1,0", MDM_CMD_SEND_TIMEOUT,
				  0, false);
		if (ret < 0) {
			LOG_ERR("AT+KUDPCFG ret:%d", ret);
			socket_put(sock);
			goto done;
		}

		/* Now wait for +KUDP_IND or +KUDP_NOTIF to ensure
		*  the socket was created */
		ret = k_sem_take(&sock->sock_send_sem, MDM_CMD_CONN_TIMEOUT);
		if (ret == 0) {
			ret = ictx.last_error;
		} else if (ret == -EAGAIN) {
			ret = -ETIMEDOUT;
		}
		if (ret < 0) {
			LOG_ERR("+KUDP_IND/NOTIF ret:%d", ret);
			socket_put(sock);
		}
	}
done:
	hl7800_unlock();
	return ret;
}

static int offload_bind(struct net_context *context,
			const struct sockaddr *addr, socklen_t addrlen)
{
	struct hl7800_socket *sock = NULL;

	if (!context) {
		return -EINVAL;
	}

	sock = (struct hl7800_socket *)context->offload_context;
	if (!sock) {
		LOG_ERR("Can't locate socket for net_ctx:%p!", context);
		return -EINVAL;
	}

	/* save bind address information */
	sock->src.sa_family = addr->sa_family;
#if defined(CONFIG_NET_IPV6)
	if (addr->sa_family == AF_INET6) {
		net_ipaddr_copy(&net_sin6(&sock->src)->sin6_addr,
				&net_sin6(addr)->sin6_addr);
		net_sin6(&sock->src)->sin6_port = net_sin6(addr)->sin6_port;
	} else
#endif
#if defined(CONFIG_NET_IPV4)
		if (addr->sa_family == AF_INET) {
		net_ipaddr_copy(&net_sin(&sock->src)->sin_addr,
				&net_sin(addr)->sin_addr);
		net_sin(&sock->src)->sin_port = net_sin(addr)->sin_port;
	} else
#endif
	{
		return -EPFNOSUPPORT;
	}

	return 0;
}

static int offload_listen(struct net_context *context, int backlog)
{
	/* NOT IMPLEMENTED */
	return -ENOTSUP;
}

static int offload_connect(struct net_context *context,
			   const struct sockaddr *addr, socklen_t addrlen,
			   net_context_connect_cb_t cb, s32_t timeout,
			   void *user_data)
{
	int ret = 0;
	int dst_port = -1;
	char cmd_cfg[sizeof("AT+KTCPCFG=#,#,\"###.###.###.###\",#####")];
	char cmd_con[sizeof("AT+KTCPCNX=##")];
	struct hl7800_socket *sock;

	if (!is_network_ready()) {
		return -EBUSY;
	}

	if (!context || !addr) {
		return -EINVAL;
	}

	sock = (struct hl7800_socket *)context->offload_context;
	if (!sock) {
		LOG_ERR("Can't locate socket for net_ctx:%p!", context);
		return -EINVAL;
	}

	if (sock->socket_id < 1) {
		LOG_ERR("Invalid socket_id(%d) for net_ctx:%p!",
			sock->socket_id, context);
		return -EINVAL;
	}

	sock->dst.sa_family = addr->sa_family;

#if defined(CONFIG_NET_IPV6)
	if (addr->sa_family == AF_INET6) {
		net_ipaddr_copy(&net_sin6(&sock->dst)->sin6_addr,
				&net_sin6(addr)->sin6_addr);
		dst_port = ntohs(net_sin6(addr)->sin6_port);
		net_sin6(&sock->dst)->sin6_port = dst_port;
	} else
#endif
#if defined(CONFIG_NET_IPV4)
		if (addr->sa_family == AF_INET) {
		net_ipaddr_copy(&net_sin(&sock->dst)->sin_addr,
				&net_sin(addr)->sin_addr);
		dst_port = ntohs(net_sin(addr)->sin_port);
		net_sin(&sock->dst)->sin_port = dst_port;
	} else
#endif
	{
		return -EINVAL;
	}

	if (dst_port < 0) {
		LOG_ERR("Invalid port: %d", dst_port);
		return -EINVAL;
	}

	hl7800_lock();

	if (sock->type == SOCK_STREAM) {
		/* Configure/create TCP connection */
		if (!sock->created) {
			snprintk(cmd_cfg, sizeof(cmd_cfg),
				 "AT+KTCPCFG=%d,%d,\"%s\",%u", 1, 0,
				 hl7800_sprint_ip_addr(addr), dst_port);
			ret = send_at_cmd(sock, cmd_cfg, MDM_CMD_SEND_TIMEOUT,
					  0, false);
			if (ret < 0) {
				LOG_ERR("AT+KTCPCFG ret:%d", ret);
				ret = -EIO;
				goto done;
			}
		}

		/* Connect to TCP */
		snprintk(cmd_con, sizeof(cmd_con), "AT+KTCPCNX=%d",
			 sock->socket_id);
		ret = send_at_cmd(sock, cmd_con, MDM_CMD_SEND_TIMEOUT, 0,
				  false);
		if (ret < 0) {
			LOG_ERR("AT+KTCPCNX ret:%d", ret);
			ret = -EIO;
			goto done;
		}
		/* Now wait for +KTCP_IND or +KTCP_NOTIF to ensure
		*  the connection succeded or failed */
		ret = k_sem_take(&sock->sock_send_sem, MDM_CMD_CONN_TIMEOUT);
		if (ret == 0) {
			ret = ictx.last_error;
		} else if (ret == -EAGAIN) {
			ret = -ETIMEDOUT;
		}
		if (ret < 0) {
			LOG_ERR("+KTCP_IND/NOTIF ret:%d", ret);
			goto done;
		} else {
			net_context_set_state(context, NET_CONTEXT_CONNECTED);
		}
	}

done:
	hl7800_unlock();

	if (cb) {
		cb(context, ret, user_data);
	}

	return ret;
}

static int offload_accept(struct net_context *context, net_tcp_accept_cb_t cb,
			  s32_t timeout, void *user_data)
{
	/* NOT IMPLEMENTED */
	return -ENOTSUP;
}

static int offload_sendto(struct net_pkt *pkt, const struct sockaddr *dst_addr,
			  socklen_t addrlen, net_context_send_cb_t cb,
			  s32_t timeout, void *user_data)
{
	struct net_context *context = net_pkt_context(pkt);
	struct hl7800_socket *sock;
	int ret, dst_port = 0;

	if (!is_network_ready()) {
		return -EBUSY;
	}

	if (!context) {
		return -EINVAL;
	}

	sock = (struct hl7800_socket *)context->offload_context;
	if (!sock) {
		LOG_ERR("Can't locate socket for net_ctx:%p!", context);
		return -EINVAL;
	}

#if defined(CONFIG_NET_IPV6)
	if (dst_addr->sa_family == AF_INET6) {
		net_ipaddr_copy(&net_sin6(&sock->dst)->sin6_addr,
				&net_sin6(dst_addr)->sin6_addr);
		dst_port = ntohs(net_sin6(dst_addr)->sin6_port);
		net_sin6(&sock->dst)->sin6_port = dst_port;
	} else
#endif
#if defined(CONFIG_NET_IPV4)
		if (dst_addr->sa_family == AF_INET) {
		net_ipaddr_copy(&net_sin(&sock->dst)->sin_addr,
				&net_sin(dst_addr)->sin_addr);
		dst_port = ntohs(net_sin(dst_addr)->sin_port);
		net_sin(&sock->dst)->sin_port = dst_port;
	} else
#endif
	{
		return -EINVAL;
	}

	hl7800_lock();

	ret = send_data(sock, pkt);

	hl7800_unlock();

	if (ret < 0) {
		LOG_ERR("send_data error: %d", ret);
	} else {
		net_pkt_unref(pkt);
	}

	if (cb) {
		cb(context, ret, user_data);
	}

	return ret;
}

static int offload_send(struct net_pkt *pkt, net_context_send_cb_t cb,
			s32_t timeout, void *user_data)
{
	struct net_context *context = net_pkt_context(pkt);
	socklen_t addrlen;

	addrlen = 0;
#if defined(CONFIG_NET_IPV6)
	if (net_pkt_family(pkt) == AF_INET6) {
		addrlen = sizeof(struct sockaddr_in6);
	} else
#endif /* CONFIG_NET_IPV6 */
#if defined(CONFIG_NET_IPV4)
		if (net_pkt_family(pkt) == AF_INET) {
		addrlen = sizeof(struct sockaddr_in);
	} else
#endif /* CONFIG_NET_IPV4 */
	{
		return -EPFNOSUPPORT;
	}

	return offload_sendto(pkt, &context->remote, addrlen, cb, timeout,
			      user_data);
}

static int offload_recv(struct net_context *context, net_context_recv_cb_t cb,
			s32_t timeout, void *user_data)
{
	struct hl7800_socket *sock;

	if (!context) {
		return -EINVAL;
	}

	sock = (struct hl7800_socket *)context->offload_context;
	if (!sock) {
		LOG_ERR("Can't locate socket for net_ctx:%p!", context);
		return -EINVAL;
	}

	sock->recv_cb = cb;
	sock->recv_user_data = user_data;

	return 0;
}

static int offload_put(struct net_context *context)
{
	struct hl7800_socket *sock;
	char cmd1[sizeof("AT+KTCPCLOSE=##")];
	char cmd2[sizeof("AT+KTCPDEL=##")];
	int ret;

	if (!context) {
		return -EINVAL;
	}

	sock = (struct hl7800_socket *)context->offload_context;
	if (!sock) {
		/* socket was already closed?  Exit quietly here. */
		return 0;
	}

	/* cancel notif work if queued */
	k_delayed_work_cancel(&sock->notif_work);

	hl7800_lock();
	/* if GPRS connection needs to be reconfigured,
	*  we dont need to issue the close command, just need to cleanup */
	if (ictx.reconfig_GPRS || !net_if_is_up(ictx.iface)) {
		LOG_DBG("Skip issuing close socket cmd");
		goto cleanup;
	}

	/* close connection */
	if (sock->type == SOCK_STREAM) {
		snprintk(cmd1, sizeof(cmd1), "AT+KTCPCLOSE=%d",
			 sock->socket_id);
		snprintk(cmd2, sizeof(cmd2), "AT+KTCPDEL=%d", sock->socket_id);
	} else {
		snprintk(cmd1, sizeof(cmd1), "AT+KUDPCLOSE=%d",
			 sock->socket_id);
	}

	ret = send_at_cmd(sock, cmd1, MDM_CMD_SEND_TIMEOUT, 0, false);
	if (ret < 0) {
		LOG_ERR("AT+K**PCLOSE ret:%d", ret);
	}

	if (sock->type == SOCK_STREAM) {
		/* delete session */
		ret = send_at_cmd(sock, cmd2, MDM_CMD_SEND_TIMEOUT, 0, false);
		if (ret < 0) {
			LOG_ERR("AT+K**PDEL ret:%d", ret);
		}
	}

cleanup:
	sock->context->connect_cb = NULL;
	sock->context->recv_cb = NULL;
	sock->context->send_cb = NULL;
	socket_put(sock);
	net_context_unref(context);
	hl7800_unlock();

	return 0;
}

static struct net_offload offload_funcs = {
	.get = offload_get,
	.bind = offload_bind,
	.listen = offload_listen,
	.connect = offload_connect,
	.accept = offload_accept,
	.send = offload_send,
	.sendto = offload_sendto,
	.recv = offload_recv,
	.put = offload_put,
};

static inline u8_t *hl7800_get_mac(struct device *dev)
{
	struct hl7800_iface_ctx *ctx = dev->driver_data;

	/* TODO: move mac_addr to Kconfig option */
	ctx->mac_addr[0] = 0x00;
	ctx->mac_addr[1] = 0x10;

	UNALIGNED_PUT(sys_cpu_to_be32(sys_rand32_get()),
		      (u32_t *)(ctx->mac_addr + 2));

	return ctx->mac_addr;
}

static int hl7800_init(struct device *dev)
{
	int i, ret = 0;

	ARG_UNUSED(dev);

	/* check for valid pinconfig */
	__ASSERT(ARRAY_SIZE(pinconfig) == MAX_MDM_CONTROL_PINS,
		 "Incorrect modem pinconfig!");

	/* Prevent the network interface from starting until the modem has been initialized
	 * because the modem may not have a valid SIM card.
	 */
	ictx.iface = net_if_get_default();
	NET_ASSERT_INFO((ictx.iface != NULL), "Invalid iface");
	net_if_flag_set(ictx.iface, NET_IF_NO_AUTO_START);

	(void)memset(&ictx, 0, sizeof(ictx));
	/* init sockets */
	for (i = 0; i < MDM_MAX_SOCKETS; i++) {
		ictx.sockets[i].socket_id = -1;
		k_work_init(&ictx.sockets[i].recv_cb_work,
			    sockreadrecv_cb_work);
		k_work_init(&ictx.sockets[i].rx_data_work,
			    sock_rx_data_cb_work);
		k_delayed_work_init(&ictx.sockets[i].notif_work,
				    sock_notif_cb_work);
		k_sem_init(&ictx.sockets[i].sock_send_sem, 0, 1);
	}
	k_sem_init(&ictx.response_sem, 0, 1);
	k_sem_init(&ictx.mdm_awake, 0, 1);

	/* initialize the work queue */
	k_work_q_start(&hl7800_workq, hl7800_workq_stack,
		       K_THREAD_STACK_SIZEOF(hl7800_workq_stack),
		       WORKQ_PRIORITY);

	k_delayed_work_init(&ictx.iface_status_work, iface_status_work_cb);
	k_delayed_work_init(&ictx.dns_work, dns_work_cb);
	k_work_init(&ictx.mdm_vgpio_work, mdm_vgpio_work_cb);
	k_delayed_work_init(&ictx.mdm_reset_work, mdm_reset_work_callback);
	ictx.last_socket_id = 0;

	/* setup port devices and pin directions */
	for (i = 0; i < MAX_MDM_CONTROL_PINS; i++) {
		ictx.gpio_port_dev[i] =
			device_get_binding(pinconfig[i].dev_name);
		if (!ictx.gpio_port_dev[i]) {
			LOG_ERR("gpio port (%s) not found!",
				pinconfig[i].dev_name);
			return -ENODEV;
		}

		ret = gpio_pin_configure(ictx.gpio_port_dev[i],
					 pinconfig[i].pin, pinconfig[i].config);
		if (ret) {
			LOG_ERR("Error configuring io %s %d err: %d!",
				pinconfig[i].dev_name, pinconfig[i].pin, ret);
			return ret;
		}
	}

	/* setup input pin callbacks */
	/* VGPIO */
	gpio_init_callback(&ictx.mdm_vgpio_cb, mdm_vgpio_callback_isr,
			   BIT(pinconfig[MDM_VGPIO].pin));
	ret = gpio_add_callback(ictx.gpio_port_dev[MDM_VGPIO],
				&ictx.mdm_vgpio_cb);
	if (ret) {
		LOG_ERR("Cannot setup vgpio callback! (%d)", ret);
		return ret;
	}
	ret = gpio_pin_enable_callback(ictx.gpio_port_dev[MDM_VGPIO],
				       pinconfig[MDM_VGPIO].pin);
	if (ret) {
		LOG_ERR("Error enabling vgpio callback! (%d)", ret);
		return ret;
	}

	/* UART DSR */
	gpio_init_callback(&ictx.mdm_uart_dsr_cb, mdm_uart_dsr_callback_isr,
			   BIT(pinconfig[MDM_UART_DSR].pin));
	ret = gpio_add_callback(ictx.gpio_port_dev[MDM_UART_DSR],
				&ictx.mdm_uart_dsr_cb);
	if (ret) {
		LOG_ERR("Cannot setup uart dsr callback! (%d)", ret);
		return ret;
	}
	ret = gpio_pin_enable_callback(ictx.gpio_port_dev[MDM_UART_DSR],
				       pinconfig[MDM_UART_DSR].pin);
	if (ret) {
		LOG_ERR("Error enabling uart dsr callback! (%d)", ret);
		return ret;
	}

	/* GPIO6 */
	gpio_init_callback(&ictx.mdm_gpio6_cb, mdm_gpio6_callback_isr,
			   BIT(pinconfig[MDM_GPIO6].pin));
	ret = gpio_add_callback(ictx.gpio_port_dev[MDM_GPIO6],
				&ictx.mdm_gpio6_cb);
	if (ret) {
		LOG_ERR("Cannot setup gpio6 callback! (%d)", ret);
		return ret;
	}
	ret = gpio_pin_enable_callback(ictx.gpio_port_dev[MDM_GPIO6],
				       pinconfig[MDM_GPIO6].pin);
	if (ret) {
		LOG_ERR("Error enabling gpio6 callback! (%d)", ret);
		return ret;
	}

	/* UART CTS */
	gpio_init_callback(&ictx.mdm_uart_cts_cb, mdm_uart_cts_callback,
			   BIT(pinconfig[MDM_UART_CTS].pin));
	ret = gpio_add_callback(ictx.gpio_port_dev[MDM_UART_CTS],
				&ictx.mdm_uart_cts_cb);
	if (ret) {
		LOG_ERR("Cannot setup uart cts callback! (%d)", ret);
		return ret;
	}
	ret = gpio_pin_enable_callback(ictx.gpio_port_dev[MDM_UART_CTS],
				       pinconfig[MDM_UART_CTS].pin);
	if (ret) {
		LOG_ERR("Error enabling uart cts callback! (%d)", ret);
		return ret;
	}

	/* Set modem data storage */
	ictx.mdm_ctx.data_manufacturer = ictx.mdm_manufacturer;
	ictx.mdm_ctx.data_model = ictx.mdm_model;
	ictx.mdm_ctx.data_revision = ictx.mdm_revision;
	ictx.mdm_ctx.data_imei = ictx.mdm_imei;

	ret = mdm_receiver_register(&ictx.mdm_ctx, MDM_UART_DEV_NAME,
				    mdm_recv_buf, sizeof(mdm_recv_buf));
	if (ret < 0) {
		LOG_ERR("Error registering modem receiver (%d)!", ret);
		return ret;
	}

	/* start RX thread */
	k_thread_name_set(
		k_thread_create(&hl7800_rx_thread, hl7800_rx_stack,
				K_THREAD_STACK_SIZEOF(hl7800_rx_stack),
				(k_thread_entry_t)hl7800_rx, NULL, NULL, NULL,
				RX_THREAD_PRIORITY, 0, K_NO_WAIT),
		"hl7800 rx");

	/* init RSSI query */
	k_delayed_work_init(&ictx.rssi_query_work, hl7800_rssi_query_work);

	ret = modem_reset_and_configure();

	return ret;
}

static void offload_iface_init(struct net_if *iface)
{
	struct device *dev = net_if_get_device(iface);
	struct hl7800_iface_ctx *ctx = dev->driver_data;

	iface->if_dev->offload = &offload_funcs;
	net_if_set_link_addr(iface, hl7800_get_mac(dev), sizeof(ctx->mac_addr),
			     NET_LINK_ETHERNET);
	ctx->iface = iface;
	ictx.initialized = true;
}

static struct net_if_api api_funcs = {
	.init = offload_iface_init,
};

NET_DEVICE_OFFLOAD_INIT(modem_hl7800, "MODEM_HL7800", hl7800_init, &ictx, NULL,
			CONFIG_MODEM_HL7800_INIT_PRIORITY, &api_funcs, MDM_MTU);
