/*
 * Copyright (c) 2022, Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <fsl_clock.h>
#include <fsl_common.h>
#include <fsl_rdc.h>
#include <fsl_iomuxc.h>
#include <init.h>
#include <kernel.h>
#include <soc.h>

#include <dt-bindings/rdc/imx_rdc.h>

#include <logging/log.h>
#define LOG_LEVEL CONFIG_SOC_LOG_LEVEL
LOG_MODULE_REGISTER(soc);

/* OSC/PLL is already initialized by ROM and Cortex-A53 (u-boot) */
static void SOC_RdcInit(void)
{
	/* Move M7 core to specific RDC domain 1 */
	rdc_domain_assignment_t assignment = { 0 };
	uint8_t domainId = 0U;

	domainId = RDC_GetCurrentMasterDomainId(RDC);
	/* Only configure the RDC if RDC peripheral write access allowed. */
	if ((0x1U & RDC_GetPeriphAccessPolicy(RDC, kRDC_Periph_RDC, domainId)) != 0U) {
		assignment.domainId = M7_DOMAIN_ID;
		RDC_SetMasterDomainAssignment(RDC, kRDC_Master_M7, &assignment);
	}

	/*
	 * The M7 core is running at domain 1, now enable the clock gate of the following IP/BUS/PLL
	 * in domain 1 in the CCM. In this way, to ensure the clock of the peripherals used by M
	 * core not be affected by A core which is running at domain 0.
	 */
	CLOCK_EnableClock(kCLOCK_Iomux);

	CLOCK_EnableClock(kCLOCK_Ipmux1);
	CLOCK_EnableClock(kCLOCK_Ipmux2);
	CLOCK_EnableClock(kCLOCK_Ipmux3);

#ifdef CONFIG_AUDIOMIX
	/* Enable the CCGR gate for AudioPLL1 in Domain 1 */
	CLOCK_ControlGate(kCLOCK_AudioPll1Gate, kCLOCK_ClockNeededAll);
	/* Enable the CCGR gate for AudioPLL2 in Domain 1 */
	CLOCK_ControlGate(kCLOCK_AudioPll2Gate, kCLOCK_ClockNeededAll);
#endif
}

#ifdef CONFIG_AUDIOMIX
/* Fractional PLLs: Fout = ((mainDiv+dsm/65536) * refSel) / (preDiv * 2^ postDiv) */
/* AUDIO PLL1 configuration */
const ccm_analog_frac_pll_config_t g_audioPll1Config = {
	.refSel = kANALOG_PllRefOsc24M, /*!< PLL reference OSC24M */
	.mainDiv = 262U,
	.dsm = 9437U,
	.preDiv = 2U,
	.postDiv = 3U, /*!< AUDIO PLL1 frequency  = 393216000HZ */
};

/* AUDIO PLL2 configuration */
const ccm_analog_frac_pll_config_t g_audioPll2Config = {
	.refSel = kANALOG_PllRefOsc24M, /*!< PLL reference OSC24M */
	.mainDiv = 361U,
	.dsm = 17511U,
	.preDiv = 3U,
	.postDiv = 3U, /*!< AUDIO PLL2 frequency  = 361267200HZ */
};

/* AUDIOMIX SAI PLL configuration */
const ccm_analog_frac_pll_config_t g_saiPLLConfig = {
	.refSel = kANALOG_PllRefOsc24M, /*!< PLL reference OSC24M */
	.mainDiv = 361U,
	.dsm = 17511U,
	.preDiv = 3U,
	.postDiv = 3U, /*!< SAI PLL frequency  = 361267200HZ */
};
#endif
#ifdef CONFIG_SYSTEM_PLL1
/* Integer PLLs: Fout = (mainDiv * refSel) / (preDiv * 2^ postDiv) */
/* SYSTEM PLL1 configuration */
const ccm_analog_integer_pll_config_t g_sysPll1Config = {
	.refSel = kANALOG_PllRefOsc24M, /*!< PLL reference OSC24M */
	.mainDiv = 400U,
	.preDiv = 3U,
	.postDiv = 2U, /*!< SYSTEM PLL1 frequency  = 800MHZ */
};
#endif

#ifdef CONFIG_SYSTEM_PLL2
/* SYSTEM PLL2 configuration */
const ccm_analog_integer_pll_config_t g_sysPll2Config = {
	.refSel = kANALOG_PllRefOsc24M, /*!< PLL reference OSC24M */
	.mainDiv = 250U,
	.preDiv = 3U,
	.postDiv = 1U, /*!< SYSTEM PLL2 frequency  = 1000MHZ */
};
#endif

#ifdef CONFIG_SYSTEM_PLL3
/* SYSTEM PLL3 configuration */
const ccm_analog_integer_pll_config_t g_sysPll3Config = {
	.refSel = kANALOG_PllRefOsc24M, /*!< PLL reference OSC24M */
	.mainDiv = 300,
	.preDiv = 3U,
	.postDiv = 2U, /*!< SYSTEM PLL3 frequency  = 600MHZ */
};
#endif

static void SOC_ClockInit(void)
{
	/*
	 * The following steps just show how to configure the PLL clock sources using the clock
	 * driver on M7 core side . Please note that the ROM has already configured the SYSTEM PLL1
	 * to 800Mhz when power up the SOC, meanwhile A core would enable SYSTEM PLL1, SYSTEM PLL2
	 * and SYSTEM PLL3 by U-Boot. Therefore, there is no need to configure the system PLL again
	 * on M7 side, otherwise it would have a risk to make the SOC hang.
	 */

	/* switch AHB NOC root to 24M first in order to configure the SYSTEM PLL1. */
	CLOCK_SetRootMux(kCLOCK_RootAhb, kCLOCK_AhbRootmuxOsc24M);

	/* switch AXI M7 root to 24M first in order to configure the SYSTEM PLL2. */
	CLOCK_SetRootMux(kCLOCK_RootM7, kCLOCK_M7RootmuxOsc24M);

#ifdef CONFIG_AUDIOMIX
	/* init AUDIO PLL1 run at 393216000HZ */
	CLOCK_InitAudioPll1(&g_audioPll1Config);
	/* init AUDIO PLL2 run at 361267200HZ */
	CLOCK_InitAudioPll2(&g_audioPll2Config);
#endif

#if CONFIG_M7_CLOCK_ROOT_24M_OSC
	CLOCK_SetRootMux(kCLOCK_RootM7, kCLOCK_M7RootmuxOsc24M);
#elif CONFIG_M7_CLOCK_ROOT_SYSTEM_PLL2_DIV5
	CLOCK_SetRootMux(kCLOCK_RootM7, kCLOCK_M7RootmuxSysPll2Div5);
#elif CONFIG_M7_CLOCK_ROOT_SYSTEM_PLL2_DIV4
	CLOCK_SetRootMux(kCLOCK_RootM7, kCLOCK_M7RootmuxSysPll2Div4);
#elif CONFIG_M7_CLOCK_ROOT_VPU_PLL
	CLOCK_SetRootMux(kCLOCK_RootM7, kCLOCK_M7RootmuxSysVpuPll);
#elif CONFIG_M7_CLOCK_ROOT_SYSTEM_PLL1
	CLOCK_SetRootMux(kCLOCK_RootM7, kCLOCK_M7RootmuxSysPll1);
#elif CONFIG_M7_CLOCK_ROOT_AUDIO_PLL1
	CLOCK_SetRootMux(kCLOCK_RootM7, kCLOCK_M7RootmuxAudioPll1);
#elif CONFIG_M7_CLOCK_ROOT_VIDEO_PLL1
	CLOCK_SetRootMux(kCLOCK_RootM7, kCLOCK_M7RootmuxVideoPll1);
#elif CONFIG_M7_CLOCK_ROOT_SYSTEM_PLL3
	CLOCK_SetRootMux(kCLOCK_RootM7, kCLOCK_M7RootmuxSysPll3);
#else
#error M7 Clock: invalid clock root configuration
#endif
	CLOCK_SetRootDivider(kCLOCK_RootM7, CONFIG_M7_PRE_ROOT_DIVIDER,
			     CONFIG_M7_POST_ROOT_DIVIDER);

#if CONFIG_AHB_CLOCK_ROOT_24M_OSC
	CLOCK_SetRootMux(kCLOCK_RootAhb, kCLOCK_AhbRootmuxOsc24M);
#elif CONFIG_AHB_CLOCK_ROOT_SYSTEM_PLL1_DIV6
	CLOCK_SetRootMux(kCLOCK_RootAhb, kCLOCK_AhbRootmuxSysPll1Div6);
#elif CONFIG_AHB_CLOCK_ROOT_SYSTEM_PLL1
	CLOCK_SetRootMux(kCLOCK_RootAhb, kCLOCK_AhbRootmuxSysPll1);
#elif CONFIG_AHB_CLOCK_ROOT_SYSTEM_PLL1_DIV2
	CLOCK_SetRootMux(kCLOCK_RootAhb, kCLOCK_AhbRootmuxSysPll1Div2);
#elif CONFIG_AHB_CLOCK_ROOT_SYSTEM_PLL2_DIV8
	CLOCK_SetRootMux(kCLOCK_RootAhb, kCLOCK_AhbRootmuxSysPll2Div8);
#elif CONFIG_AHB_CLOCK_ROOT_SYSTEM_PLL3
	CLOCK_SetRootMux(kCLOCK_RootAhb, kCLOCK_AhbRootmuxSysPll3);
#elif CONFIG_AHB_CLOCK_ROOT_AUDIO_PLL1
	CLOCK_SetRootMux(kCLOCK_RootAhb, kCLOCK_AhbRootmuxAudioPll1);
#elif CONFIG_AHB_CLOCK_ROOT_VIDEO_PLL1
	CLOCK_SetRootMux(kCLOCK_RootAhb, kCLOCK_AhbRootmuxVideoPll1);
#else
#error AHB Clock: invalid clock root configuration
#endif
	CLOCK_SetRootDivider(kCLOCK_RootAhb, CONFIG_AHB_PRE_ROOT_DIVIDER,
			     CONFIG_AHB_POST_ROOT_DIVIDER);

#ifdef CONFIG_AUDIOMIX
#if CONFIG_AUDIO_AHB_CLOCK_ROOT_24M_OSC
	CLOCK_SetRootMux(kCLOCK_RootAudioAhb, kCLOCK_AudioAhbRootmuxOsc24M);
#elif CONFIG_AUDIO_AHB_CLOCK_ROOT_SYSTEM_PLL2_DIV2
	CLOCK_SetRootMux(kCLOCK_RootAudioAhb, kCLOCK_AudioAhbRootmuxSysPll2Div2);
#elif CONFIG_AUDIO_AHB_CLOCK_ROOT_SYSTEM_PLL1
	CLOCK_SetRootMux(kCLOCK_RootAudioAhb, kCLOCK_AudioAhbRootmuxSysPll1);
#elif CONFIG_AUDIO_AHB_CLOCK_ROOT_SYSTEM_PLL2
	CLOCK_SetRootMux(kCLOCK_RootAudioAhb, kCLOCK_AudioAhbRootmuxSysPll2);
#elif CONFIG_AUDIO_AHB_CLOCK_ROOT_SYSTEM_PLL2_DIV6
	CLOCK_SetRootMux(kCLOCK_RootAudioAhb, kCLOCK_AudioAhbRootmuxSysPll2Div6);
#elif CONFIG_AUDIO_AHB_CLOCK_ROOT_SYSTEM_PLL3
	CLOCK_SetRootMux(kCLOCK_RootAudioAhb, kCLOCK_AudioAhbRootmuxSysPll3);
#elif CONFIG_AUDIO_AHB_CLOCK_ROOT_AUDIO_PLL1
	CLOCK_SetRootMux(kCLOCK_RootAudioAhb, kCLOCK_AudioAhbRootmuxAudioPll1);
#elif CONFIG_AUDIO_AHB_CLOCK_ROOT_VIDEO_PLL1
	CLOCK_SetRootMux(kCLOCK_RootAudioAhb, kCLOCK_AudioAhbRootmuxVideoPll1);
#else
#error Audio AHB Clock: invalid clock root configuration
#endif
	CLOCK_SetRootDivider(kCLOCK_RootAudioAhb, CONFIG_AUDIO_AHB_PRE_ROOT_DIVIDER,
			     CONFIG_AUDIO_AHB_POST_ROOT_DIVIDER);
#endif

	CLOCK_EnableClock(kCLOCK_Rdc); /* Enable RDC clock */
	CLOCK_EnableClock(kCLOCK_Ocram); /* Enable Ocram clock */

	/* The purpose to enable the following modules clock is to make sure the M7 core could work
	 * normally when A53 core enters the low power status.
	 */
	CLOCK_EnableClock(kCLOCK_Sim_m);
	CLOCK_EnableClock(kCLOCK_Sim_main);
	CLOCK_EnableClock(kCLOCK_Sim_s);
	CLOCK_EnableClock(kCLOCK_Sim_wakeup);
	CLOCK_EnableClock(kCLOCK_Debug);
	CLOCK_EnableClock(kCLOCK_Dram);
	CLOCK_EnableClock(kCLOCK_Sec_Debug);

#ifdef CONFIG_AUDIOMIX
	/* Enable Audio clock to power on the audiomix domain*/
	CLOCK_EnableClock(kCLOCK_Audio);

	/* Power up the audiomix domain by M7 core.*/

	/* Map the audiomix domain to M7 */
	GPC->PGC_CPU_M7_MAPPING |= 1U << GPC_PGC_CPU_M7_MAPPING_AUDIOMIX_DOMAIN_SHIFT;
	/* Software request to trigger power up the domain */
	GPC->PU_PGC_SW_PUP_REQ |= 1U << GPC_PU_PGC_SW_PUP_REQ_AUDIOMIX_SW_PUP_REQ_SHIFT;

	/*
	 * Wait for the GPC_PU_PGC_SW_PUP_REQ_AUDIOMIX_SW_PUP_REQ bit to be self-cleared after power
	 * up
	 */
	while (GPC->PU_PGC_SW_PUP_REQ & (1U << GPC_PU_PGC_SW_PUP_REQ_AUDIOMIX_SW_PUP_REQ_SHIFT)) {
		;
	}
	/*
	 * Perform handshake to make sure the NOC bus is ready after power up of the AUDIOMIX domain
	 */
	GPC->PU_PWRHSK |= 1U << GPC_PU_PWRHSK_GPC_AUDIOMIX_NOC_PWRDNREQN_SHIFT;
	while (!(GPC->PU_PWRHSK & (1U << GPC_PU_PWRHSK_GPC_AUDIOMIX_PWRDNACKN_SHIFT))) {
		;
	}

	/* init SAI PLL run at 361267200HZ */
	AUDIOMIX_InitAudioPll(AUDIOMIX, &g_saiPLLConfig);
#endif

	/* Update core clock */
	SystemCoreClockUpdate();
}

static int summit_som8m_init(const struct device *arg)
{
	ARG_UNUSED(arg);

	/* SoC specific RDC settings */
	SOC_RdcInit();

	/* SoC specific Clock settings */
	SOC_ClockInit();

	return 0;
}

SYS_INIT(summit_som8m_init, PRE_KERNEL_1, 0);
