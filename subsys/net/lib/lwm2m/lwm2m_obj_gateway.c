/**
 * @file lwm2m_gateway.c
 * @brief
 *
 * Copyright (c) 2021 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME net_lwm2m_gateway
#define LOG_LEVEL CONFIG_LWM2M_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <string.h>
#include <init.h>

#include "lwm2m_object.h"
#include "lwm2m_engine.h"
#include "lwm2m_resource_ids.h"
#include "lwm2m_obj_gateway.h"

#if defined(CONFIG_LWM2M_GATEWAY_VERSION_3_0)
#define GATEWAY_VERSION_MAJOR 3
#define GATEWAY_VERSION_MINOR 0
#define GATEWAY_MAX_ID 5
#else
#define GATEWAY_VERSION_MAJOR 2
#define GATEWAY_VERSION_MINOR 0
#define GATEWAY_MAX_ID 4
#endif

#define MAX_INSTANCE_COUNT CONFIG_LWM2M_GATEWAY_MAX_INSTANCES

/*
 * Calculate resource instances as follows:
 * start with GATEWAY_MAX_ID
 * subtract EXEC resources (1)
 */
#define RESOURCE_INSTANCE_COUNT (GATEWAY_MAX_ID - 1)

struct lwm2m_gw_obj {
	char device_id[CONFIG_LWM2M_GATEWAY_DEVICE_ID_MAX_STR_SIZE];
	char prefix[CONFIG_LWM2M_GATEWAY_PREFIX_MAX_STR_SIZE];
	char iot_device_objects[CONFIG_LWM2M_GATEWAY_IOT_DEVICE_OBJECTS_MAX_STR_SIZE];
#if defined(CONFIG_LWM2M_GATEWAY_VERSION_3_0)
	int8_t rssi;
#endif
};

static struct lwm2m_gw_obj device_table[MAX_INSTANCE_COUNT];
static struct lwm2m_engine_obj lwm2m_gw;
static struct lwm2m_engine_obj_field fields[] = {
	OBJ_FIELD_DATA(LWM2M_GATEWAY_DEVICE_RID, R, STRING),
	OBJ_FIELD_DATA(LWM2M_GATEWAY_PREFIX_RID, RW, STRING),
	OBJ_FIELD_DATA(LWM2M_GATEWAY_IOT_DEVICE_OBJECTS_RID, R, STRING),
#if defined(CONFIG_LWM2M_GATEWAY_VERSION_3_0)
	OBJ_FIELD_DATA(LWM2M_GATEWAY_RSSI_RID, R_OPT, S8),
#endif
};

static struct lwm2m_engine_obj_inst inst[MAX_INSTANCE_COUNT];
static struct lwm2m_engine_res res[MAX_INSTANCE_COUNT][GATEWAY_MAX_ID];
static struct lwm2m_engine_res_inst res_inst[MAX_INSTANCE_COUNT][RESOURCE_INSTANCE_COUNT];

static struct lwm2m_engine_obj_inst *lwm2m_gw_create(uint16_t obj_inst_id)
{
	int index, i = 0, j = 0;

	/* Check that there is no other instance with this ID */
	for (index = 0; index < MAX_INSTANCE_COUNT; index++) {
		if (inst[index].obj && inst[index].obj_inst_id == obj_inst_id) {
			LOG_ERR("Can not create instance - "
				"already existing: %u",
				obj_inst_id);
			return NULL;
		}
	}

	for (index = 0; index < MAX_INSTANCE_COUNT; index++) {
		if (!inst[index].obj) {
			break;
		}
	}

	if (index >= MAX_INSTANCE_COUNT) {
		LOG_ERR("Can not create instance - no more room: %u", obj_inst_id);
		return NULL;
	}

	/* Set default values */
	strncpy(device_table[index].device_id, CONFIG_LWM2M_GATEWAY_DEFAULT_DEVICE_ID,
		CONFIG_LWM2M_GATEWAY_DEVICE_ID_MAX_STR_SIZE);
	strncpy(device_table[index].prefix, CONFIG_LWM2M_GATEWAY_DEFAULT_DEVICE_PREFIX,
		CONFIG_LWM2M_GATEWAY_PREFIX_MAX_STR_SIZE);
	strncpy(device_table[index].iot_device_objects,
		CONFIG_LWM2M_GATEWAY_DEFAULT_IOT_DEVICE_OBJECTS,
		CONFIG_LWM2M_GATEWAY_IOT_DEVICE_OBJECTS_MAX_STR_SIZE);
#if defined(CONFIG_LWM2M_GATEWAY_VERSION_3_0)
	device_table[index].rssi = LWM2M_GATEWAY_INVALID_RSSI;
#endif

	(void)memset(res[index], 0, sizeof(res[index][0]) * ARRAY_SIZE(res[index]));
	init_res_instance(res_inst[index], ARRAY_SIZE(res_inst[index]));

	/* initialize instance resource data */
	INIT_OBJ_RES_DATA(LWM2M_GATEWAY_DEVICE_RID, res[index], i, res_inst[index], j,
			  device_table[index].device_id,
			  CONFIG_LWM2M_GATEWAY_DEVICE_ID_MAX_STR_SIZE);
	INIT_OBJ_RES_DATA(LWM2M_GATEWAY_PREFIX_RID, res[index], i, res_inst[index], j,
			  device_table[index].prefix, CONFIG_LWM2M_GATEWAY_PREFIX_MAX_STR_SIZE);
	INIT_OBJ_RES_DATA(LWM2M_GATEWAY_IOT_DEVICE_OBJECTS_RID, res[index], i, res_inst[index], j,
			  device_table[index].iot_device_objects,
			  sizeof(device_table[index].iot_device_objects));
#if defined(CONFIG_LWM2M_GATEWAY_VERSION_3_0)
	INIT_OBJ_RES_DATA(LWM2M_GATEWAY_RSSI_RID, res[index], i, res_inst[index], j, &device_table[index].rssi,
			  sizeof(device_table[index].rssi));
#endif

	inst[index].resources = res[index];
	inst[index].resource_count = i;
	LOG_DBG("Created LWM2M gateway instance: %d", obj_inst_id);
	return &inst[index];
}

static int lwm2m_gw_init(const struct device *dev)
{
	int ret = 0;

	/* initialize the LwM2M Gateway field data */
	lwm2m_gw.obj_id = LWM2M_OBJECT_GATEWAY_ID;
	lwm2m_gw.version_major = GATEWAY_VERSION_MAJOR;
	lwm2m_gw.version_minor = GATEWAY_VERSION_MINOR;
	lwm2m_gw.is_core = true;
	lwm2m_gw.fields = fields;
	lwm2m_gw.field_count = ARRAY_SIZE(fields);
	lwm2m_gw.max_instance_count = MAX_INSTANCE_COUNT;
	lwm2m_gw.create_cb = lwm2m_gw_create;
	lwm2m_register_obj(&lwm2m_gw);
	return ret;
}

SYS_INIT(lwm2m_gw_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
