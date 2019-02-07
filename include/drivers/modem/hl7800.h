/** @file
 * @brief HL7800 modem header file.
 *
 * Allows an application to control the HL7800 modem.
 *
 */

/*
 * Copyright (c) 2019 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MODEM_HL7800_H_
#define ZEPHYR_INCLUDE_DRIVERS_MODEM_HL7800_H_

#include <stdlib.h>
#include <kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

void mdm_hl7800_power_off(void);
void mdm_hl7800_reset(void);
void mdm_hl7800_wakeup(bool);
void mdm_hl7800_send_at_cmd(const u8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MODEM_HL7800_H_ */
