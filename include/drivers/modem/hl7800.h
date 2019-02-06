/** @file
 * @brief HL7800 modem header file.
 *
 * Allows an application to control the HL7800 modem.
 *
 * Copyright (c) 2019 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MODEM_HL7800_H_
#define ZEPHYR_INCLUDE_DRIVERS_MODEM_HL7800_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Power off the HL7800
 * 
 * @return s32_t 0 for success
 */
s32_t mdm_hl7800_power_off(void);

/**
 * @brief Reset the HL7800
 * 
 * @return s32_t 0 for success
 */
s32_t mdm_hl7800_reset(void);

/**
 * @brief Control the wake signals to the HL7800
 * 
 * @param awake True to keep the HL7800 awake, False to allow sleep
 */
void mdm_hl7800_wakeup(bool awake);

/**
 * @brief Send an AT command to the HL7800
 * 
 * @param data AT command string
 * @return s32_t 0 for success
 */
s32_t mdm_hl7800_send_at_cmd(const u8_t *data);

/**
 * @brief Get the signal quality of the HL7800
 * 
 * @param rsrp Reference Signals Received Power (dBm)
 *             Range = -140 dBm to -44 dBm
 * @param sinr Signal to Interference plus Noise Ratio (dBm)
 *             Range = -128 dBm to 40dBm
 */
void mdm_hl7800_get_signal_quality(int *rsrp, int *sinr);

/**
 * @brief Get the SIM card ICCID
 * 
 */
char *mdm_hl7800_get_iccid(void);

/**
 * @brief Get the HL7800 serial number
 * 
 */
char *mdm_hl7800_get_sn(void);


#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MODEM_HL7800_H_ */
