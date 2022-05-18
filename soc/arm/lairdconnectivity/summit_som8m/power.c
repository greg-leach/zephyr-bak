/*
 * Copyright (c) 2022, Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr.h>
#include <device.h>
#include <pm/pm.h>
#include <fsl_gpc.h>

#include <logging/log.h>
LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

#define PowerUpSlot (5U)
#define PowerDnSlot (6U)

__weak void gpc_configure_interrupts(void)
{
	uint32_t irq;

	/* Enable GPT interrupt source for GPC - this is system timer */
	irq = DT_IRQN(DT_INST(0, nxp_gpt_hw_timer));
	GPC_EnableIRQ(GPC, irq);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(mailbox0), okay)
	/* Enable MU interrupt source for GPC */
	irq = DT_IRQN(DT_INST(0, nxp_imx_mu));
	GPC_EnableIRQ(GPC, irq);
#endif
}

/* Initializes configuration for the GPC */
static void gpc_init(void)
{
	GPC_Init(GPC, PowerUpSlot, PowerDnSlot);
	/* Setup GPC interrupts */
	gpc_configure_interrupts();
}

/**
 * SOC specific low power mode implementation
 * Drop to lowest power state possible given system's request
 */
__weak void pm_power_state_set(struct pm_state_info info)
{
	/* Extract target GPC mode from the substate ID */
	uint32_t target_mode = (uint32_t)info.substate_id;
	/* Read current GPC mode */
	uint32_t current_mode = GPC_GetLpmMode(GPC);

	irq_unlock(0);

	if (target_mode == current_mode) {
		return;
	}

	gpc_lpm_config_t config;
	config.enCpuClk = false;
	config.enFastWakeUp = false;
	config.enDsmMask = false;
	config.enWfiMask = false;
	config.enVirtualPGCPowerdown = true;
	config.enVirtualPGCPowerup = true;

	switch (target_mode) {
	case kGPC_RunMode:
		LOG_DBG("Switch to GPC Run Mode requested");
		GPC->LPCR_M7 = GPC->LPCR_M7 & (~GPC_LPCR_M7_LPM0_MASK);
		return;
	case kGPC_WaitMode:
		LOG_DBG("Switch to GPC Wait Mode requested");
		GPC_EnterWaitMode(GPC, &config);
		break;
	case kGPC_StopMode:
		LOG_DBG("Switch to GPC Stop Mode requested");
		GPC_EnterStopMode(GPC, &config);
		break;
	default:
		LOG_DBG("Switch to unknown GPC Mode requested");
		return;
	}

	/* WFI instruction will start entry into WAIT/STOP mode */
	LOG_DBG("Entering LPM via WFI");
	__DSB();
	__ISB();
	__WFI();
}

__weak void pm_power_state_exit_post_ops(struct pm_state_info info)
{
	ARG_UNUSED(info);
	LOG_DBG("Exiting LPM");
}

/* Initialize Power */
static int power_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	/* Initialize general power controller */
	gpc_init();
	LOG_DBG("GPC initialized");
	return 0;
}

SYS_INIT(power_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
