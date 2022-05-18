/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_imx_ccm
#include <errno.h>
#include <soc.h>
#include <drivers/clock_control.h>
#include <dt-bindings/clock/imx_ccm.h>
#include <fsl_clock.h>

#define LOG_LEVEL CONFIG_CLOCK_CONTROL_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(clock_control);

#ifdef CONFIG_SPI_MCUX_LPSPI
static const clock_name_t lpspi_clocks[] = {
	kCLOCK_Usb1PllPfd1Clk,
	kCLOCK_Usb1PllPfd0Clk,
	kCLOCK_SysPllClk,
	kCLOCK_SysPllPfd2Clk,
};
#endif

#ifdef CONFIG_SOC_PART_NUMBER_IMX8ML_M7
/*
 * As an effort to reduce power usage, PLL gates are only enabled when a peripheral submits a
 * 'request'. As an example, an I2C peripheral's clock root can be sourced from the 24MHz
 * oscillator, System PLL 1, System PLL 2, System PLL 3, Audio PLL 1, Audio PLL 2, or Video PLL 1.
 * If the clock root is sourced from the 24MHz oscillator, no PLLs are needed, so no gates are
 * enabled. Since there are many peripherals and each can be set to a different clock source, a
 * count is kept of the requests for each PLL type. If the count for a PLL is greater than one, the
 * corresponding gate is enabled, and conversely, if the count is zero, the corresponding gate is
 * disabled.
 *
 * Note: A peripheral's clock root is configured in the devicetree.
 */
K_MUTEX_DEFINE(pll_request_mutex);

/**
 * @brief Enumeration of PLL names
 *
 */
typedef enum _pll_name {
	SYSTEM_PLL1 = 0,
	SYSTEM_PLL2 = 1,
	SYSTEM_PLL3 = 2,
	AUDIO_PLL12 = 3,
	VIDEO_PLL1 = 4
} pll_name_t;

/**
 * @brief PLL requests
 *
 */
typedef struct _pll_requests {
	uint8_t system_pll_1;
	uint8_t system_pll_2;
	uint8_t system_pll_3;
	uint8_t audio_plls;
	uint8_t video_pll;
} pll_requests_t;

static pll_requests_t pll_requests = {0};

/**
 * @brief Increment the request count on a given PLL type
 *
 * @param pll_name The PLL to add a request for
 */
void mcux_ccm_increment_pll_request(pll_name_t pll_name)
{
	k_mutex_lock(&pll_request_mutex, K_FOREVER);

	switch (pll_name) {
	case SYSTEM_PLL1:
		pll_requests.system_pll_1++;
		CLOCK_ControlGate(kCLOCK_SysPll1Gate, kCLOCK_ClockNeededAll);
		break;
	case SYSTEM_PLL2:
		pll_requests.system_pll_2++;
		CLOCK_ControlGate(kCLOCK_SysPll2Gate, kCLOCK_ClockNeededAll);
		break;
	case SYSTEM_PLL3:
		pll_requests.system_pll_3++;
		CLOCK_ControlGate(kCLOCK_SysPll3Gate, kCLOCK_ClockNeededAll);
		break;
	case AUDIO_PLL12:
		pll_requests.audio_plls++;
		CLOCK_ControlGate(kCLOCK_AudioPll1Gate, kCLOCK_ClockNeededAll);
		CLOCK_ControlGate(kCLOCK_AudioPll2Gate, kCLOCK_ClockNeededAll);
		break;
	case VIDEO_PLL1:
		pll_requests.video_pll++;
		CLOCK_ControlGate(kCLOCK_VideoPll1Gate, kCLOCK_ClockNeededAll);
		break;
	default:
		k_mutex_unlock(&pll_request_mutex);
		LOG_ERR("Invalid PLL specified");
		return;
	}

	k_mutex_unlock(&pll_request_mutex);
}

/**
 * @brief Decrement the request count on a given PLL type
 *
 * @param pll_name The PLL to remove a request for
 */
void mcux_ccm_decrement_pll_request(pll_name_t pll_name)
{
	k_mutex_lock(&pll_request_mutex, K_FOREVER);

	switch (pll_name) {
	case SYSTEM_PLL1:
		if (--pll_requests.system_pll_1 == 0) {
			CLOCK_ControlGate(kCLOCK_SysPll1Gate, kCLOCK_ClockNotNeeded);
		}
		break;
	case SYSTEM_PLL2:
		if (--pll_requests.system_pll_2 == 0) {
			CLOCK_ControlGate(kCLOCK_SysPll2Gate, kCLOCK_ClockNotNeeded);
		}
		break;
	case SYSTEM_PLL3:
		if (--pll_requests.system_pll_3 == 0) {
			CLOCK_ControlGate(kCLOCK_SysPll3Gate, kCLOCK_ClockNotNeeded);
		}
		break;
	case AUDIO_PLL12:
		if (--pll_requests.audio_plls == 0) {
			CLOCK_ControlGate(kCLOCK_AudioPll1Gate, kCLOCK_ClockNotNeeded);
			CLOCK_ControlGate(kCLOCK_AudioPll2Gate, kCLOCK_ClockNotNeeded);
		}
		break;
	case VIDEO_PLL1:
		if (--pll_requests.video_pll == 0) {
			CLOCK_ControlGate(kCLOCK_VideoPll1Gate, kCLOCK_ClockNotNeeded);
		}
		break;
	default:
		k_mutex_unlock(&pll_request_mutex);
		LOG_ERR("Invalid PLL specified");
		return;
	}

	k_mutex_unlock(&pll_request_mutex);
}

#if defined(CONFIG_SPI_MCUX_ECSPI)
/**
 * @brief Handle a PLL enable request for an ECSPI peripheral instance. If the given clock rootmux
 * does not require a PLL, no request is made.
 *
 * @param clock_rootmux The ECSPI clock rootmux used by the peripheral instance
 */
static void mcux_ccm_ecspi_increment_pll_request(clock_rootmux_ecspi_clk_sel_t clock_rootmux)
{
	switch (clock_rootmux) {
	case kCLOCK_EcspiRootmuxSysPll1Div20:
	case kCLOCK_EcspiRootmuxSysPll1Div5:
	case kCLOCK_EcspiRootmuxSysPll1:
		mcux_ccm_increment_pll_request(SYSTEM_PLL1);
		break;
	case kCLOCK_EcspiRootmuxSysPll2Div5:
	case kCLOCK_EcspiRootmuxSysPll2Div4:
		mcux_ccm_increment_pll_request(SYSTEM_PLL2);
		break;
	case kCLOCK_EcspiRootmuxSysPll3:
		mcux_ccm_increment_pll_request(SYSTEM_PLL3);
		break;
	case kCLOCK_EcspiRootmuxAudioPll2:
		mcux_ccm_increment_pll_request(AUDIO_PLL12);
		break;
	case kCLOCK_EcspiRootmuxOsc24M:
	default:
		break;
	}
}
#endif

#if defined(CONFIG_CAN_MCUX_FLEXCAN)
/**
 * @brief Handle a PLL enable request for a FlexCAN peripheral instance. If the given clock rootmux
 * does not require a PLL, no request is made.
 *
 * @param clock_rootmux The FlexCAN clock rootmux used by the peripheral instance
 */
static void mcux_ccm_flexcan_increment_pll_request(clock_rootmux_flexcan_clk_sel_t clock_rootmux)
{
	switch (clock_rootmux) {
	case kCLOCK_FlexCanRootmuxSysPll1Div20:
	case kCLOCK_FlexCanRootmuxSysPll1Div5:
	case kCLOCK_FlexCanRootmuxSysPll1:
		mcux_ccm_increment_pll_request(SYSTEM_PLL1);
		break;
	case kCLOCK_FlexCanRootmuxSysPll2Div5:
	case kCLOCK_FlexCanRootmuxSysPll2Div4:
		mcux_ccm_increment_pll_request(SYSTEM_PLL2);
		break;
	case kCLOCK_FlexCanRootmuxSysPll3:
		mcux_ccm_increment_pll_request(SYSTEM_PLL3);
		break;
	case kCLOCK_FlexCanRootmuxAudioPll2:
		mcux_ccm_increment_pll_request(AUDIO_PLL12);
		break;
	case kCLOCK_FlexCanRootmuxOsc24M:
	default:
		break;
	}
}
#endif

#if defined(CONFIG_MEMC_MCUX_FLEXSPI)
/**
 * @brief Handle a PLL enable request for a FlexSPI peripheral instance. If the given clock rootmux
 * does not require a PLL, no request is made.
 *
 * @param clock_rootmux The FlexSPI clock rootmux used by the peripheral instance
 */
static void mcux_ccm_flexspi_increment_pll_request(clock_rootmux_qspi_clk_sel_t clock_rootmux)
{
	switch (clock_rootmux) {
	case kCLOCK_QspiRootmuxSysPll1Div2:
	case kCLOCK_QspiRootmuxSysPll1Div3:
	case kCLOCK_QspiRootmuxSysPll1Div8:
		mcux_ccm_increment_pll_request(SYSTEM_PLL1);
		break;
	case kCLOCK_QspiRootmuxSysPll2Div3:
	case kCLOCK_QspiRootmuxSysPll2Div2:
		mcux_ccm_increment_pll_request(SYSTEM_PLL2);
		break;
	case kCLOCK_QspiRootmuxSysPll3:
		mcux_ccm_increment_pll_request(SYSTEM_PLL3);
		break;
	case kCLOCK_QspiRootmuxAudioPll2:
		mcux_ccm_increment_pll_request(AUDIO_PLL12);
		break;
	case kCLOCK_QspiRootmuxOsc24M:
	default:
		break;
	}
}
#endif

#if defined(CONFIG_COUNTER_MCUX_GPT)
/**
 * @brief Handle a PLL enable request for a GPT peripheral instance. If the given clock rootmux
 * does not require a PLL, no request is made.
 *
 * @param clock_rootmux The GPT clock rootmux used by the peripheral instance
 */
static void mcux_ccm_gpt_increment_pll_request(clock_rootmux_gpt_t clock_rootmux)
{
	switch (clock_rootmux) {
	case kCLOCK_GptRootmuxSysPll1Div2:
	case kCLOCK_GptRootmuxSysPll1Div20:
	case kCLOCK_GptRootmuxSystemPll1Div10:
		mcux_ccm_increment_pll_request(SYSTEM_PLL1);
		break;
	case kCLOCK_GptRootmuxSystemPll2Div10:
		mcux_ccm_increment_pll_request(SYSTEM_PLL2);
		break;
	case kCLOCK_GptRootmuxAudioPll1:
		mcux_ccm_increment_pll_request(AUDIO_PLL12);
		break;
	case kCLOCK_GptRootmuxVideoPll1:
		mcux_ccm_increment_pll_request(VIDEO_PLL1);
		break;
	case kCLOCK_GptRootmuxOsc24M:
	case kCLOCK_GptRootmuxExtClk123:
	default:
		break;
	}
}
#endif

#if defined(CONFIG_I2C_MCUX_II2C)
/**
 * @brief Handle a PLL enable request for an I2C peripheral instance. If the given clock rootmux
 * does not require a PLL, no request is made.
 *
 * @param clock_rootmux The I2C clock rootmux used by the peripheral instance
 */
static void mcux_ccm_i2c_increment_pll_request(clock_rootmux_i2c_clk_sel_t clock_rootmux)
{
	switch (clock_rootmux) {
	case kCLOCK_I2cRootmuxSysPll1Div5:
	case kCLOCK_I2cRootmuxSysPll1Div6:
		mcux_ccm_increment_pll_request(SYSTEM_PLL1);
		break;
	case kCLOCK_I2cRootmuxSysPll2Div20:
		mcux_ccm_increment_pll_request(SYSTEM_PLL2);
		break;
	case kCLOCK_I2cRootmuxSysPll3:
		mcux_ccm_increment_pll_request(SYSTEM_PLL3);
		break;
	case kCLOCK_I2cRootmuxAudioPll1:
	case kCLOCK_I2cRootmuxAudioPll2:
		mcux_ccm_increment_pll_request(AUDIO_PLL12);
		break;
	case kCLOCK_I2cRootmuxVideoPll1:
		mcux_ccm_increment_pll_request(VIDEO_PLL1);
		break;
	case kCLOCK_I2cRootmuxOsc24M:
	default:
		break;
	}
}
#endif

#if defined(CONFIG_PWM_IMX)
/**
 * @brief Handle a PLL enable request for a PWM peripheral instance. If the given clock rootmux
 * does not require a PLL, no request is made.
 *
 * @param clock_rootmux The PWM clock rootmux used by the peripheral instance
 */
static void mcux_ccm_pwm_increment_pll_request(clock_rootmux_Pwm_clk_sel_t clock_rootmux)
{
	switch (clock_rootmux) {
	case kCLOCK_PwmRootmuxSysPll1Div5:
	case kCLOCK_PwmRootmuxSysPll1Div20:
	case kCLOCK_PwmRootmuxSystemPll1Div10:
		mcux_ccm_increment_pll_request(SYSTEM_PLL1);
		break;
	case kCLOCK_PwmRootmuxSysPll2Div10:
		mcux_ccm_increment_pll_request(SYSTEM_PLL2);
		break;
	case kCLOCK_PwmRootmuxSystemPll3:
		mcux_ccm_increment_pll_request(SYSTEM_PLL3);
		break;
	case kCLOCK_PwmRootmuxVideoPll1:
		mcux_ccm_increment_pll_request(VIDEO_PLL1);
		break;
	case kCLOCK_PwmRootmuxOsc24M:
	case kCLOCK_PwmRootmuxExtClk12:
	default:
		break;
	}
}
#endif

#if defined(CONFIG_UART_MCUX_IUART)
/**
 * @brief Handle a PLL enable request for a UART peripheral instance. If the given clock rootmux
 * does not require a PLL, no request is made.
 *
 * @param clock_rootmux The UART clock rootmux used by the peripheral instance
 */
static void mcux_ccm_uart_increment_pll_request(clock_rootmux_uart_clk_sel_t clock_rootmux)
{
	switch (clock_rootmux) {
	case kCLOCK_UartRootmuxSysPll1Div10:
		mcux_ccm_increment_pll_request(SYSTEM_PLL1);
		break;
	case kCLOCK_UartRootmuxSysPll2Div5:
	case kCLOCK_UartRootmuxSysPll2Div10:
		mcux_ccm_increment_pll_request(SYSTEM_PLL2);
		break;
	case kCLOCK_UartRootmuxSysPll3:
		mcux_ccm_increment_pll_request(SYSTEM_PLL3);
		break;
	case kCLOCK_UartRootmuxAudioPll2:
		mcux_ccm_increment_pll_request(AUDIO_PLL12);
		break;
	case kCLOCK_UartRootmuxOsc24M:
	case kCLOCK_UartRootmuxExtClk2:
	case kCLOCK_UartRootmuxExtClk34:
	default:
		break;
	}
}
#endif

#if defined(CONFIG_WDT_MCUX_IMX_WDOG)
/**
 * @brief Handle a PLL enable request for a watchdog peripheral instance. If the given clock rootmux
 * does not require a PLL, no request is made.
 *
 * @param clock_rootmux The watchdog clock rootmux used by the peripheral instance
 */
static void mcux_ccm_wdog_increment_pll_request(clock_rootmux_wdog_clk_sel_t clock_rootmux)
{
	switch (clock_rootmux) {
	case kCLOCK_WdogRootmuxSysPll1Div6:
	case kCLOCK_WdogRootmuxSysPll1Div5:
	case kCLOCK_WdogRootmuxSystemPll1Div10:
		mcux_ccm_increment_pll_request(SYSTEM_PLL1);
		break;
	case kCLOCK_WdogRootmuxSystemPll2Div8:
	case kCLOCK_WdogRootmuxSystemPll2Div6:
		mcux_ccm_increment_pll_request(SYSTEM_PLL2);
		break;
	case kCLOCK_WdogRootmuxSystemPll3:
		mcux_ccm_increment_pll_request(SYSTEM_PLL3);
		break;
	case kCLOCK_WdogRootmuxOsc24M:
	case kCLOCK_WdogRootmuxVpuPll:
	default:
		break;
	}
}
#endif

/**
 * @brief Update the root mux, pre-divider, and post-divider for a given peripheral instance, and
 * enable the clcok if necessary.
 *
 * @param clock_root_control Name of the clock root for the peripheral instance
 * @param clock_ip_name Name of the clock for the peripheral instance
 * @param clock_rootmux Desired clock root mux configuration for the peripheral instance
 * @param pre Desired clock pre-divider for the peripheral instance
 * @param post Desired clock post-divider for the peripheral instance
 * @param enable If true, enable the clock
 */
static void mcux_ccm_configure_clock(clock_root_control_t clock_root_control,
				     clock_ip_name_t clock_ip_name, uint32_t clock_rootmux,
				     uint32_t pre, uint32_t post, bool enable)
{
	CLOCK_UpdateRoot(clock_root_control, clock_rootmux, pre, post);
	if (enable) {
		CLOCK_EnableClock(clock_ip_name);
	}
}
#endif

#ifdef CONFIG_I2C_MCUX_II2C
/**
 * @brief Get the current clock frequency of an I2C peripheral instance
 *
 * @param i2cRootClk Name of the clock root for the I2C peripheral instance
 * @return uint32_t Current clock frequency (Hz) or 0 on error
 */
static uint32_t CLOCK_GetI2cFreq(clock_root_control_t i2cRootClk)
{
	uint32_t freq;
	uint32_t pre = CLOCK_GetRootPreDivider(i2cRootClk);
	uint32_t post = CLOCK_GetRootPostDivider(i2cRootClk);

	switch (CLOCK_GetRootMux(i2cRootClk)) {
	case (uint32_t)kCLOCK_I2cRootmuxOsc24M:
		freq = OSC24M_CLK_FREQ;
		break;
	case (uint32_t)kCLOCK_I2cRootmuxSysPll1Div5:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / 5U;
		break;
	case (uint32_t)kCLOCK_I2cRootmuxSysPll2Div20:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll2Ctrl) / 20U;
		break;
	case (uint32_t)kCLOCK_I2cRootmuxSysPll3:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll3Ctrl);
		break;
	case (uint32_t)kCLOCK_I2cRootmuxAudioPll1:
		freq = CLOCK_GetPllFreq(kCLOCK_AudioPll1Ctrl);
		break;
	case (uint32_t)kCLOCK_I2cRootmuxVideoPll1:
		freq = CLOCK_GetPllFreq(kCLOCK_VideoPll1Ctrl);
		break;
	case (uint32_t)kCLOCK_I2cRootmuxAudioPll2:
		freq = CLOCK_GetPllFreq(kCLOCK_AudioPll2Ctrl);
		break;
	case (uint32_t)kCLOCK_I2cRootmuxSysPll1Div6:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / 6U;
		break;
	default:
		freq = 0U;
		break;
	}

	return freq / pre / post;
}
#endif

#ifdef CONFIG_COUNTER_MCUX_GPT
/**
 * @brief Get the current clock frequency of a GPT peripheral instance
 *
 * @param gptRootClk Name of the clock root for the GPT peripheral instance
 * @return uint32_t Current clock frequency (Hz) or 0 on error
 */
static uint32_t CLOCK_GetGptFreq(clock_root_control_t gptRootClk)
{
	uint32_t freq;
	uint32_t pre = CLOCK_GetRootPreDivider(gptRootClk);
	uint32_t post = CLOCK_GetRootPostDivider(gptRootClk);

	switch (CLOCK_GetRootMux(gptRootClk)) {
	case (uint32_t)kCLOCK_GptRootmuxOsc24M:
		freq = OSC24M_CLK_FREQ;
		break;
	case (uint32_t)kCLOCK_GptRootmuxSystemPll2Div10:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll2Ctrl) / 10U;
		break;
	case (uint32_t)kCLOCK_GptRootmuxSysPll1Div2:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / 2U;
		break;
	case (uint32_t)kCLOCK_GptRootmuxSysPll1Div20:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / 20U;
		break;
	case (uint32_t)kCLOCK_GptRootmuxVideoPll1:
		freq = CLOCK_GetPllFreq(kCLOCK_VideoPll1Ctrl);
		break;
	case (uint32_t)kCLOCK_GptRootmuxSystemPll1Div10:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / 10U;
		break;
	case (uint32_t)kCLOCK_GptRootmuxAudioPll1:
		freq = CLOCK_GetPllFreq(kCLOCK_AudioPll1Ctrl);
		break;
	/*
	 * If the clock root is set to an external clock, there's no way to know
	 * what the frequency is, so just return 0.
	 */
	case (uint32_t)kCLOCK_GptRootmuxExtClk123:
	default:
		freq = 0U;
		break;
	}

	return freq / pre / post;
}
#endif

#ifdef CONFIG_CAN_MCUX_FLEXCAN
/**
 * @brief Get the current clock frequency of a FlexCAN peripheral instance
 *
 * @param canRootClk Name of the clock root for the FlexCAN peripheral instance
 * @return uint32_t Current clock frequency (Hz) or 0 on error
 */
static uint32_t CLOCK_GetCanFreq(clock_root_control_t canRootClk)
{
	uint32_t freq;
	uint32_t pre = CLOCK_GetRootPreDivider(canRootClk);
	uint32_t post = CLOCK_GetRootPostDivider(canRootClk);

	switch (CLOCK_GetRootMux(canRootClk)) {
	case (uint32_t)kCLOCK_FlexCanRootmuxOsc24M:
		freq = OSC24M_CLK_FREQ;
		break;
	case (uint32_t)kCLOCK_FlexCanRootmuxSysPll2Div5:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll2Ctrl) / 5U;
		break;
	case (uint32_t)kCLOCK_FlexCanRootmuxSysPll1Div20:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / 20U;
		break;
	case (uint32_t)kCLOCK_FlexCanRootmuxSysPll1Div5:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / 5U;
		break;
	case (uint32_t)kCLOCK_FlexCanRootmuxSysPll1:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl);
		break;
	case (uint32_t)kCLOCK_FlexCanRootmuxSysPll3:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll3Ctrl);
		break;
	case (uint32_t)kCLOCK_FlexCanRootmuxSysPll2Div4:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll2Ctrl) / 4U;
		break;
	case (uint32_t)kCLOCK_FlexCanRootmuxAudioPll2:
		freq = CLOCK_GetPllFreq(kCLOCK_AudioPll2Ctrl);
		break;
	default:
		freq = 0U;
		break;
	}

	return freq / pre / post;
}
#endif

#ifdef CONFIG_SPI_MCUX_ECSPI
/**
 * @brief Get the current clock frequency of an ECSPI peripheral instance
 *
 * @param ecspiRootClk Name of the clock root for the ECSPI peripheral instance
 * @return uint32_t Current clock frequency (Hz) or 0 on error
 */
static uint32_t CLOCK_GetEcspiFreq(clock_root_control_t ecspiRootClk)
{
	uint32_t freq;
	uint32_t pre = CLOCK_GetRootPreDivider(ecspiRootClk);
	uint32_t post = CLOCK_GetRootPostDivider(ecspiRootClk);

	switch (CLOCK_GetRootMux(ecspiRootClk)) {
	case (uint32_t)kCLOCK_EcspiRootmuxOsc24M:
		freq = OSC24M_CLK_FREQ;
		break;
	case (uint32_t)kCLOCK_EcspiRootmuxSysPll2Div5:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll2Ctrl) / 5U;
		break;
	case (uint32_t)kCLOCK_EcspiRootmuxSysPll1Div20:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / 20U;
		break;
	case (uint32_t)kCLOCK_EcspiRootmuxSysPll1Div5:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / 5U;
		break;
	case (uint32_t)kCLOCK_EcspiRootmuxSysPll1:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl);
		break;
	case (uint32_t)kCLOCK_EcspiRootmuxSysPll3:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll3Ctrl);
		break;
	case (uint32_t)kCLOCK_EcspiRootmuxSysPll2Div4:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll2Ctrl) / 4U;
		break;
	case (uint32_t)kCLOCK_EcspiRootmuxAudioPll2:
		freq = CLOCK_GetPllFreq(kCLOCK_AudioPll2Ctrl);
		break;
	default:
		freq = 0U;
		break;
	}

	return freq / pre / post;
}
#endif

#ifdef CONFIG_UART_MCUX_IUART
/**
 * @brief Get the current clock frequency of a UART peripheral instance
 *
 * @param uartRootClk Name of the clock root for the UART peripheral instance
 * @return uint32_t Current clock frequency (Hz) or 0 on error
 */
static uint32_t CLOCK_GetUartFreq(clock_root_control_t uartRootClk)
{
	uint32_t freq;
	uint32_t pre = CLOCK_GetRootPreDivider(uartRootClk);
	uint32_t post = CLOCK_GetRootPostDivider(uartRootClk);

	switch (CLOCK_GetRootMux(uartRootClk)) {
	case (uint32_t)kCLOCK_UartRootmuxOsc24M:
		freq = OSC24M_CLK_FREQ;
		break;
	case (uint32_t)kCLOCK_UartRootmuxSysPll1Div10:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / 10U;
		break;
	case (uint32_t)kCLOCK_UartRootmuxSysPll2Div5:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll2Ctrl) / 5U;
		break;
	case (uint32_t)kCLOCK_UartRootmuxSysPll2Div10:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll2Ctrl) / 10U;
		break;
	case (uint32_t)kCLOCK_UartRootmuxSysPll3:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll3Ctrl);
		break;
	case (uint32_t)kCLOCK_UartRootmuxAudioPll2:
		freq = CLOCK_GetPllFreq(kCLOCK_AudioPll2Ctrl);
		break;
	/*
	 * If the clock root is set to an external clock, there's no way to know
	 * what the frequency is, so just return 0.
	 */
	case (uint32_t)kCLOCK_UartRootmuxExtClk2:
	case (uint32_t)kCLOCK_UartRootmuxExtClk34:
	default:
		freq = 0U;
		break;
	}

	return freq / pre / post;
}
#endif

#ifdef CONFIG_PWM_IMX
/**
 * @brief Get the current clock frequency of a PWM peripheral instance
 *
 * @param pwmRootClk Name of the clock root for the PWM peripheral instance
 * @return uint32_t Current clock frequency (Hz) or 0 on error
 */
static uint32_t CLOCK_GetPwmFreq(clock_root_control_t pwmRootClk)
{
	uint32_t freq;
	uint32_t pre = CLOCK_GetRootPreDivider(pwmRootClk);
	uint32_t post = CLOCK_GetRootPostDivider(pwmRootClk);

	switch (CLOCK_GetRootMux(pwmRootClk)) {
	case (uint32_t)kCLOCK_PwmRootmuxOsc24M:
		freq = OSC24M_CLK_FREQ;
		break;
	case (uint32_t)kCLOCK_PwmRootmuxSysPll2Div10:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll2Ctrl) / 10U;
		break;
	case (uint32_t)kCLOCK_PwmRootmuxSysPll1Div5:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / 5U;
		break;
	case (uint32_t)kCLOCK_PwmRootmuxSysPll1Div20:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / 20U;
		break;
	case (uint32_t)kCLOCK_PwmRootmuxSystemPll3:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll3Ctrl);
		break;
	case (uint32_t)kCLOCK_PwmRootmuxSystemPll1Div10:
		freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / 10U;
		break;
	case (uint32_t)kCLOCK_PwmRootmuxVideoPll1:
		freq = CLOCK_GetPllFreq(kCLOCK_VideoPll1Ctrl);
		break;
	/*
	 * If the clock root is set to an external clock, there's no way to know
	 * what the frequency is, so just return 0.
	 */
	case (uint32_t)kCLOCK_PwmRootmuxExtClk12:
	default:
		freq = 0U;
		break;
	}

	return freq / pre / post;
}
#endif

static int mcux_ccm_on(const struct device *dev,
			      clock_control_subsys_t sub_system)
{
	uint32_t clock_name = (uint32_t)sub_system;

	switch (clock_name) {
#ifdef CONFIG_SPI_MCUX_ECSPI
#ifdef CONFIG_SOC_PART_NUMBER_IMX8ML_M7
	case IMX_CCM_ECSPI1_CLK:
		mcux_ccm_ecspi_increment_pll_request(DT_PROP(DT_NODELABEL(ecspi1), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootEcspi1, kCLOCK_Ecspi1,
					 DT_PROP(DT_NODELABEL(ecspi1), clock_rootmux),
					 DT_PROP(DT_NODELABEL(ecspi1), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(ecspi1), clock_root_post_divider),
					 true);
		return 0;
	case IMX_CCM_ECSPI2_CLK:
		mcux_ccm_ecspi_increment_pll_request(DT_PROP(DT_NODELABEL(ecspi2), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootEcspi2, kCLOCK_Ecspi2,
					 DT_PROP(DT_NODELABEL(ecspi2), clock_rootmux),
					 DT_PROP(DT_NODELABEL(ecspi2), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(ecspi2), clock_root_post_divider),
					 true);
		return 0;
	case IMX_CCM_ECSPI3_CLK:
		mcux_ccm_ecspi_increment_pll_request(DT_PROP(DT_NODELABEL(ecspi3), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootEcspi3, kCLOCK_Ecspi3,
					 DT_PROP(DT_NODELABEL(ecspi3), clock_rootmux),
					 DT_PROP(DT_NODELABEL(ecspi3), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(ecspi3), clock_root_post_divider),
					 true);
		return 0;
#endif
#endif

#ifdef CONFIG_CAN_MCUX_FLEXCAN
#ifdef CONFIG_SOC_PART_NUMBER_IMX8ML_M7
	case IMX_CCM_CAN1_CLK:
		mcux_ccm_flexcan_increment_pll_request(
			DT_PROP(DT_NODELABEL(flexcan1), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootFlexCan1, kCLOCK_Can1,
					 DT_PROP(DT_NODELABEL(flexcan1), clock_rootmux),
					 DT_PROP(DT_NODELABEL(flexcan1), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(flexcan1), clock_root_post_divider),
					 true);
		return 0;
	case IMX_CCM_CAN2_CLK:
		mcux_ccm_flexcan_increment_pll_request(
			DT_PROP(DT_NODELABEL(flexcan2), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootFlexCan2, kCLOCK_Can2,
					 DT_PROP(DT_NODELABEL(flexcan2), clock_rootmux),
					 DT_PROP(DT_NODELABEL(flexcan2), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(flexcan2), clock_root_post_divider),
					 true);
		return 0;
#endif
#endif

#ifdef CONFIG_MEMC_MCUX_FLEXSPI
#ifdef CONFIG_SOC_PART_NUMBER_IMX8ML_M7
	case IMX_CCM_FLEXSPI_CLK:
		mcux_ccm_flexspi_increment_pll_request(
			DT_PROP(DT_NODELABEL(flexspi), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootQspi, kCLOCK_Qspi,
					 DT_PROP(DT_NODELABEL(flexspi), clock_rootmux),
					 DT_PROP(DT_NODELABEL(flexspi), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(flexspi), clock_root_post_divider),
					 true);
		return 0;
#endif
#endif

#ifdef CONFIG_COUNTER_MCUX_GPT
#ifdef CONFIG_SOC_PART_NUMBER_IMX8ML_M7
	case IMX_CCM_GPT1_CLK:
		/* GPT1 is used as the hw timer for tickless support */
		return 0;
	case IMX_CCM_GPT2_CLK:
		mcux_ccm_gpt_increment_pll_request(DT_PROP(DT_NODELABEL(gpt2), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootGpt2, kCLOCK_Gpt2,
					 DT_PROP(DT_NODELABEL(gpt2), clock_rootmux),
					 DT_PROP(DT_NODELABEL(gpt2), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(gpt2), clock_root_post_divider),
					 false);
		return 0;
	case IMX_CCM_GPT3_CLK:
		mcux_ccm_gpt_increment_pll_request(DT_PROP(DT_NODELABEL(gpt3), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootGpt3, kCLOCK_Gpt3,
					 DT_PROP(DT_NODELABEL(gpt3), clock_rootmux),
					 DT_PROP(DT_NODELABEL(gpt3), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(gpt3), clock_root_post_divider),
					 true);
		return 0;
	case IMX_CCM_GPT4_CLK:
		mcux_ccm_gpt_increment_pll_request(DT_PROP(DT_NODELABEL(gpt4), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootGpt4, kCLOCK_Gpt4,
					 DT_PROP(DT_NODELABEL(gpt4), clock_rootmux),
					 DT_PROP(DT_NODELABEL(gpt4), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(gpt4), clock_root_post_divider),
					 true);
		return 0;
	case IMX_CCM_GPT5_CLK:
		mcux_ccm_gpt_increment_pll_request(DT_PROP(DT_NODELABEL(gpt5), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootGpt5, kCLOCK_Gpt5,
					 DT_PROP(DT_NODELABEL(gpt5), clock_rootmux),
					 DT_PROP(DT_NODELABEL(gpt5), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(gpt5), clock_root_post_divider),
					 true);
		return 0;
	case IMX_CCM_GPT6_CLK:
		mcux_ccm_gpt_increment_pll_request(DT_PROP(DT_NODELABEL(gpt6), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootGpt6, kCLOCK_Gpt6,
					 DT_PROP(DT_NODELABEL(gpt6), clock_rootmux),
					 DT_PROP(DT_NODELABEL(gpt6), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(gpt6), clock_root_post_divider),
					 true);
		return 0;
#endif
#endif

#ifdef CONFIG_I2C_MCUX_II2C
#ifdef CONFIG_SOC_PART_NUMBER_IMX8ML_M7
	case IMX_CCM_I2C1_CLK:
		mcux_ccm_i2c_increment_pll_request(DT_PROP(DT_NODELABEL(i2c1), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootI2c1, kCLOCK_I2c1,
					 DT_PROP(DT_NODELABEL(i2c1), clock_rootmux),
					 DT_PROP(DT_NODELABEL(i2c1), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(i2c1), clock_root_post_divider),
					 true);
		return 0;
	case IMX_CCM_I2C2_CLK:
		mcux_ccm_i2c_increment_pll_request(DT_PROP(DT_NODELABEL(i2c2), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootI2c2, kCLOCK_I2c2,
					 DT_PROP(DT_NODELABEL(i2c2), clock_rootmux),
					 DT_PROP(DT_NODELABEL(i2c2), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(i2c2), clock_root_post_divider),
					 true);
		return 0;
	case IMX_CCM_I2C3_CLK:
		mcux_ccm_i2c_increment_pll_request(DT_PROP(DT_NODELABEL(i2c3), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootI2c3, kCLOCK_I2c3,
					 DT_PROP(DT_NODELABEL(i2c3), clock_rootmux),
					 DT_PROP(DT_NODELABEL(i2c3), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(i2c3), clock_root_post_divider),
					 true);
		return 0;
	case IMX_CCM_I2C4_CLK:
		mcux_ccm_i2c_increment_pll_request(DT_PROP(DT_NODELABEL(i2c4), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootI2c4, kCLOCK_I2c4,
					 DT_PROP(DT_NODELABEL(i2c4), clock_rootmux),
					 DT_PROP(DT_NODELABEL(i2c4), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(i2c4), clock_root_post_divider),
					 true);
		return 0;
	case IMX_CCM_I2C5_CLK:
		mcux_ccm_i2c_increment_pll_request(DT_PROP(DT_NODELABEL(i2c5), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootI2c5, kCLOCK_I2c5,
					 DT_PROP(DT_NODELABEL(i2c5), clock_rootmux),
					 DT_PROP(DT_NODELABEL(i2c5), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(i2c5), clock_root_post_divider),
					 true);
		return 0;
	case IMX_CCM_I2C6_CLK:
		mcux_ccm_i2c_increment_pll_request(DT_PROP(DT_NODELABEL(i2c6), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootI2c6, kCLOCK_I2c6,
					 DT_PROP(DT_NODELABEL(i2c6), clock_rootmux),
					 DT_PROP(DT_NODELABEL(i2c6), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(i2c6), clock_root_post_divider),
					 true);
		return 0;
#endif
#endif

#ifdef CONFIG_PWM_IMX
#ifdef CONFIG_SOC_PART_NUMBER_IMX8ML_M7
	case IMX_CCM_PWM1_CLK:
		mcux_ccm_pwm_increment_pll_request(DT_PROP(DT_NODELABEL(pwm1), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootPwm1, kCLOCK_Pwm1,
					 DT_PROP(DT_NODELABEL(pwm1), clock_rootmux),
					 DT_PROP(DT_NODELABEL(pwm1), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(pwm1), clock_root_post_divider),
					 true);
		return 0;
	case IMX_CCM_PWM2_CLK:
		mcux_ccm_pwm_increment_pll_request(DT_PROP(DT_NODELABEL(pwm2), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootPwm2, kCLOCK_Pwm2,
					 DT_PROP(DT_NODELABEL(pwm2), clock_rootmux),
					 DT_PROP(DT_NODELABEL(pwm2), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(pwm2), clock_root_post_divider),
					 true);
		return 0;
	case IMX_CCM_PWM3_CLK:
		mcux_ccm_pwm_increment_pll_request(DT_PROP(DT_NODELABEL(pwm3), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootPwm3, kCLOCK_Pwm3,
					 DT_PROP(DT_NODELABEL(pwm3), clock_rootmux),
					 DT_PROP(DT_NODELABEL(pwm3), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(pwm3), clock_root_post_divider),
					 true);
		return 0;
	case IMX_CCM_PWM4_CLK:
		mcux_ccm_pwm_increment_pll_request(DT_PROP(DT_NODELABEL(pwm4), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootPwm4, kCLOCK_Pwm4,
					 DT_PROP(DT_NODELABEL(pwm4), clock_rootmux),
					 DT_PROP(DT_NODELABEL(pwm4), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(pwm4), clock_root_post_divider),
					 true);
		return 0;
#endif
#endif

#ifdef CONFIG_UART_MCUX_IUART
#ifdef CONFIG_SOC_PART_NUMBER_IMX8ML_M7
	case IMX_CCM_UART4_CLK:
		mcux_ccm_uart_increment_pll_request(DT_PROP(DT_NODELABEL(uart4), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootUart4, kCLOCK_Uart4,
					 DT_PROP(DT_NODELABEL(uart4), clock_rootmux),
					 DT_PROP(DT_NODELABEL(uart4), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(uart4), clock_root_post_divider),
					 true);
		return 0;
#endif
#endif

#ifdef CONFIG_WDT_MCUX_IMX_WDOG
#ifdef CONFIG_SOC_PART_NUMBER_IMX8ML_M7
	case IMX_CCM_WDOG1_CLK:
		mcux_ccm_wdog_increment_pll_request(DT_PROP(DT_NODELABEL(wdog1), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootWdog, kCLOCK_Wdog1,
					 DT_PROP(DT_NODELABEL(wdog1), clock_rootmux),
					 DT_PROP(DT_NODELABEL(wdog1), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(wdog1), clock_root_post_divider),
					 true);
		return 0;
	case IMX_CCM_WDOG2_CLK:
		mcux_ccm_wdog_increment_pll_request(DT_PROP(DT_NODELABEL(wdog2), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootWdog, kCLOCK_Wdog2,
					 DT_PROP(DT_NODELABEL(wdog2), clock_rootmux),
					 DT_PROP(DT_NODELABEL(wdog2), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(wdog2), clock_root_post_divider),
					 true);
		return 0;
	case IMX_CCM_WDOG3_CLK:
		mcux_ccm_wdog_increment_pll_request(DT_PROP(DT_NODELABEL(wdog3), clock_rootmux));
		mcux_ccm_configure_clock(kCLOCK_RootWdog, kCLOCK_Wdog3,
					 DT_PROP(DT_NODELABEL(wdog3), clock_rootmux),
					 DT_PROP(DT_NODELABEL(wdog3), clock_root_pre_divider),
					 DT_PROP(DT_NODELABEL(wdog3), clock_root_post_divider),
					 true);
		return 0;
#endif
#endif

	default:
		return 0;
	}
}

static int mcux_ccm_off(const struct device *dev,
			       clock_control_subsys_t sub_system)
{
	return 0;
}

static int mcux_ccm_get_subsys_rate(const struct device *dev,
				    clock_control_subsys_t sub_system,
				    uint32_t *rate)
{
	uint32_t clock_name = (uint32_t) sub_system;

	switch (clock_name) {

#ifdef CONFIG_I2C_MCUX_LPI2C
	case IMX_CCM_LPI2C_CLK:
		if (CLOCK_GetMux(kCLOCK_Lpi2cMux) == 0) {
			*rate = CLOCK_GetPllFreq(kCLOCK_PllUsb1) / 8
				/ (CLOCK_GetDiv(kCLOCK_Lpi2cDiv) + 1);
		} else {
			*rate = CLOCK_GetOscFreq()
				/ (CLOCK_GetDiv(kCLOCK_Lpi2cDiv) + 1);
		}

		break;
#endif

#ifdef CONFIG_I2C_MCUX_II2C
	case IMX_CCM_I2C1_CLK:
		*rate = CLOCK_GetI2cFreq(kCLOCK_RootI2c1);
		break;
	case IMX_CCM_I2C2_CLK:
		*rate = CLOCK_GetI2cFreq(kCLOCK_RootI2c2);
		break;
	case IMX_CCM_I2C3_CLK:
		*rate = CLOCK_GetI2cFreq(kCLOCK_RootI2c3);
		break;
	case IMX_CCM_I2C4_CLK:
		*rate = CLOCK_GetI2cFreq(kCLOCK_RootI2c4);
		break;
	case IMX_CCM_I2C5_CLK:
		*rate = CLOCK_GetI2cFreq(kCLOCK_RootI2c5);
		break;
	case IMX_CCM_I2C6_CLK:
		*rate = CLOCK_GetI2cFreq(kCLOCK_RootI2c6);
		break;
#endif

#ifdef CONFIG_SPI_MCUX_LPSPI
	case IMX_CCM_LPSPI_CLK:
	{
		uint32_t lpspi_mux = CLOCK_GetMux(kCLOCK_LpspiMux);
		clock_name_t lpspi_clock = lpspi_clocks[lpspi_mux];

		*rate = CLOCK_GetFreq(lpspi_clock)
			/ (CLOCK_GetDiv(kCLOCK_LpspiDiv) + 1);
		break;
	}
#endif

#ifdef CONFIG_SPI_MCUX_ECSPI
	case IMX_CCM_ECSPI1_CLK:
		*rate = CLOCK_GetEcspiFreq(kCLOCK_RootEcspi1);
		break;
	case IMX_CCM_ECSPI2_CLK:
		*rate = CLOCK_GetEcspiFreq(kCLOCK_RootEcspi2);
		break;
	case IMX_CCM_ECSPI3_CLK:
		*rate = CLOCK_GetEcspiFreq(kCLOCK_RootEcspi3);
		break;
#endif

#ifdef CONFIG_UART_MCUX_LPUART
	case IMX_CCM_LPUART_CLK:
		if (CLOCK_GetMux(kCLOCK_UartMux) == 0) {
			*rate = CLOCK_GetPllFreq(kCLOCK_PllUsb1) / 6
				/ (CLOCK_GetDiv(kCLOCK_UartDiv) + 1);
		} else {
			*rate = CLOCK_GetOscFreq()
				/ (CLOCK_GetDiv(kCLOCK_UartDiv) + 1);
		}

		break;
#endif

#ifdef CONFIG_PWM_IMX
#ifdef CONFIG_SOC_PART_NUMBER_IMX8ML_M7
	case IMX_CCM_PWM1_CLK:
		*rate = CLOCK_GetPwmFreq(kCLOCK_RootPwm1);
		break;
	case IMX_CCM_PWM2_CLK:
		*rate = CLOCK_GetPwmFreq(kCLOCK_RootPwm2);
		break;
	case IMX_CCM_PWM3_CLK:
		*rate = CLOCK_GetPwmFreq(kCLOCK_RootPwm3);
		break;
	case IMX_CCM_PWM4_CLK:
		*rate = CLOCK_GetPwmFreq(kCLOCK_RootPwm4);
		break;
#endif
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(usdhc1), okay) && CONFIG_DISK_DRIVER_SDMMC
	case IMX_CCM_USDHC1_CLK:
		*rate = CLOCK_GetSysPfdFreq(kCLOCK_Pfd0) /
				(CLOCK_GetDiv(kCLOCK_Usdhc1Div) + 1U);
		break;
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(usdhc2), okay) && CONFIG_DISK_DRIVER_SDMMC
	case IMX_CCM_USDHC2_CLK:
		*rate = CLOCK_GetSysPfdFreq(kCLOCK_Pfd0) /
				(CLOCK_GetDiv(kCLOCK_Usdhc2Div) + 1U);
		break;
#endif

#ifdef CONFIG_DMA_MCUX_EDMA
	case IMX_CCM_EDMA_CLK:
		*rate = CLOCK_GetIpgFreq();
		break;
#endif

#ifdef CONFIG_UART_MCUX_IUART
#ifdef CONFIG_SOC_PART_NUMBER_IMX8ML_M7
	case IMX_CCM_UART2_CLK:
		*rate = CLOCK_GetUartFreq(kCLOCK_RootUart2);
		break;
	case IMX_CCM_UART3_CLK:
		*rate = CLOCK_GetUartFreq(kCLOCK_RootUart3);
		break;
	case IMX_CCM_UART4_CLK:
		*rate = CLOCK_GetUartFreq(kCLOCK_RootUart4);
		break;
#else
	case IMX_CCM_UART_CLK:
		*rate = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) /
				(CLOCK_GetRootPreDivider(kCLOCK_RootUart4)) /
				(CLOCK_GetRootPostDivider(kCLOCK_RootUart4)) /
				10;
		break;
#endif
#endif

#ifdef CONFIG_CAN_MCUX_FLEXCAN
#ifdef CONFIG_SOC_PART_NUMBER_IMX8ML_M7
	case IMX_CCM_CAN1_CLK:
		*rate = CLOCK_GetCanFreq(kCLOCK_RootFlexCan1);
		break;
	case IMX_CCM_CAN2_CLK:
		*rate = CLOCK_GetCanFreq(kCLOCK_RootFlexCan2);
		break;
#else
	case IMX_CCM_CAN_CLK:
	{
		uint32_t can_mux = CLOCK_GetMux(kCLOCK_CanMux);

		if (can_mux == 0) {
			*rate = CLOCK_GetPllFreq(kCLOCK_PllUsb1) / 8
				/ (CLOCK_GetDiv(kCLOCK_CanDiv) + 1);
		} else if  (can_mux == 1) {
			*rate = CLOCK_GetOscFreq()
				/ (CLOCK_GetDiv(kCLOCK_CanDiv) + 1);
		} else {
			*rate = CLOCK_GetPllFreq(kCLOCK_PllUsb1) / 6
				/ (CLOCK_GetDiv(kCLOCK_CanDiv) + 1);
		}
	} break;
#endif
#endif

#ifdef CONFIG_COUNTER_MCUX_GPT
#ifdef CONFIG_SOC_PART_NUMBER_IMX8ML_M7
	case IMX_CCM_GPT1_CLK:
		*rate = CLOCK_GetGptFreq(kCLOCK_RootGpt1);
		break;
	case IMX_CCM_GPT2_CLK:
		*rate = CLOCK_GetGptFreq(kCLOCK_RootGpt2);
		break;
	case IMX_CCM_GPT3_CLK:
		*rate = CLOCK_GetGptFreq(kCLOCK_RootGpt3);
		break;
	case IMX_CCM_GPT4_CLK:
		*rate = CLOCK_GetGptFreq(kCLOCK_RootGpt4);
		break;
	case IMX_CCM_GPT5_CLK:
		*rate = CLOCK_GetGptFreq(kCLOCK_RootGpt5);
		break;
	case IMX_CCM_GPT6_CLK:
		*rate = CLOCK_GetGptFreq(kCLOCK_RootGpt6);
		break;
#else
	case IMX_CCM_GPT_CLK:
		*rate = CLOCK_GetFreq(kCLOCK_PerClk);
		break;
#endif
#endif

	}

	return 0;
}

static int mcux_ccm_init(const struct device *dev)
{
	return 0;
}

static const struct clock_control_driver_api mcux_ccm_driver_api = {
	.on = mcux_ccm_on,
	.off = mcux_ccm_off,
	.get_rate = mcux_ccm_get_subsys_rate,
};

DEVICE_DT_INST_DEFINE(0,
		    &mcux_ccm_init,
		    NULL,
		    NULL, NULL,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &mcux_ccm_driver_api);
