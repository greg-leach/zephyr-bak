/*
 * Copyright (c) 2022, Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC__H_
#define _SOC__H_

#include <sys/util.h>

#ifndef _ASMLANGUAGE

#include <fsl_common.h>
#ifdef CONFIG_AUDIOMIX
#include <fsl_audiomix.h>
#endif

/* Add include for DTS generated information */
#include <devicetree.h>

#ifdef __cplusplus
extern "C" {
#endif

#endif /* !_ASMLANGUAGE */

#ifdef __cplusplus
}
#endif

#endif /* _SOC__H_ */
