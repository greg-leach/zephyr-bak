/*
 * Copyright (c) 2022, Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef CONFIG_SOC_PART_NUMBER_IMX8ML_M7
#define DT_DRV_COMPAT nxp_imx8mp_mcux_ecspi
#else
#define DT_DRV_COMPAT nxp_mcux_ecspi
#endif

#include <errno.h>
#include <drivers/spi.h>
#include <drivers/clock_control.h>
#include <fsl_ecspi.h>
#include <devicetree.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(spi_mcux_ecspi, CONFIG_SPI_LOG_LEVEL);

#include "spi_context.h"

#define CHIP_SELECT_COUNT 1
#define MAX_DATA_WIDTH 64

struct spi_mcux_config {
	ECSPI_Type *base;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	void (*irq_config_func)(const struct device *dev);
	uint8_t chip_select_delay;
	uint16_t sample_period;
	ecspi_sample_period_clock_source_t sample_period_clock_source;
	uint8_t burst_length;
	ecspi_clock_inactive_state_t clock_inactive_state;
	ecspi_data_line_inactive_state_t data_line_inactive_state;
	ecspi_chip_select_active_state_t chip_select_active_state;
};

struct spi_mcux_data {
	const struct device *dev;
	ecspi_master_handle_t handle;
	struct spi_context ctx;
	size_t transfer_len;
};

static void spi_mcux_transfer_next_packet(const struct device *dev)
{
	const struct spi_mcux_config *config = dev->config;
	struct spi_mcux_data *data = dev->data;
	ECSPI_Type *base = config->base;
	struct spi_context *ctx = &data->ctx;
	ecspi_transfer_t transfer;
	status_t status;

	if ((ctx->tx_len == 0) && (ctx->rx_len == 0)) {
		/* nothing left to rx or tx, we're done! */
		spi_context_cs_control(&data->ctx, false);
		spi_context_complete(&data->ctx, 0);
		return;
	}

	transfer.channel = kECSPI_Channel0;

	if (ctx->tx_len == 0) {
		/* rx only, nothing to tx */
		transfer.txData = NULL;
		transfer.rxData = (uint32_t *)ctx->rx_buf;
		transfer.dataSize = ctx->rx_len;
	} else if (ctx->rx_len == 0) {
		/* tx only, nothing to rx */
		transfer.txData = (uint32_t *)ctx->tx_buf;
		transfer.rxData = NULL;
		transfer.dataSize = ctx->tx_len;
	} else if (ctx->tx_len == ctx->rx_len) {
		/* rx and tx are the same length */
		transfer.txData = (uint32_t *)ctx->tx_buf;
		transfer.rxData = (uint32_t *)ctx->rx_buf;
		transfer.dataSize = ctx->tx_len;
	} else if (ctx->tx_len > ctx->rx_len) {
		/* Break up the tx into multiple transfers so we don't have to
		 * rx into a longer intermediate buffer. Leave chip select
		 * active between transfers.
		 */
		transfer.txData = (uint32_t *)ctx->tx_buf;
		transfer.rxData = (uint32_t *)ctx->rx_buf;
		transfer.dataSize = ctx->rx_len;
	} else {
		/* Break up the rx into multiple transfers so we don't have to
		 * tx from a longer intermediate buffer. Leave chip select
		 * active between transfers.
		 */
		transfer.txData = (uint32_t *)ctx->tx_buf;
		transfer.rxData = (uint32_t *)ctx->rx_buf;
		transfer.dataSize = ctx->tx_len;
	}

	data->transfer_len = transfer.dataSize;

	status = ECSPI_MasterTransferNonBlocking(base, &data->handle, &transfer);
	if (status != kStatus_Success) {
		LOG_ERR("Transfer could not start");
	}
}

static void spi_mcux_isr(const struct device *dev)
{
	const struct spi_mcux_config *config = dev->config;
	struct spi_mcux_data *data = dev->data;
	ECSPI_Type *base = config->base;

	ECSPI_MasterTransferHandleIRQ(base, &data->handle);
}

static void spi_mcux_master_transfer_callback(ECSPI_Type *base, ecspi_master_handle_t *handle,
					      status_t status, void *userData)
{
	struct spi_mcux_data *data = userData;

	spi_context_update_tx(&data->ctx, 1, data->transfer_len);
	spi_context_update_rx(&data->ctx, 1, data->transfer_len);

	spi_mcux_transfer_next_packet(data->dev);
}

static int spi_mcux_configure(const struct device *dev, const struct spi_config *spi_cfg)
{
	const struct spi_mcux_config *config = dev->config;
	struct spi_mcux_data *data = dev->data;
	ECSPI_Type *base = config->base;
	ecspi_master_config_t master_config;
	ecspi_master_handle_t master_handle = data->handle;
	uint32_t clock_freq;
	uint32_t word_size;

	if (spi_context_configured(&data->ctx, spi_cfg)) {
		/* This configuration is already in use */
		return 0;
	}

	if (spi_cfg->operation & SPI_HALF_DUPLEX) {
		LOG_ERR("Half-duplex not supported");
		return -ENOTSUP;
	}

	word_size = SPI_WORD_SIZE_GET(spi_cfg->operation);
	if (word_size > MAX_DATA_WIDTH) {
		LOG_ERR("Word size %d is greater than %d", word_size, MAX_DATA_WIDTH);
		return -EINVAL;
	}

	/* Only master mode is currently supported */
	if (SPI_OP_MODE_GET(spi_cfg->operation) != SPI_OP_MODE_MASTER) {
		LOG_ERR("Slave mode not supported");
		return -ENOTSUP;
	}

	ECSPI_MasterGetDefaultConfig(&master_config);

	if (spi_cfg->slave > CHIP_SELECT_COUNT) {
		LOG_ERR("Slave %d is greater than %d", spi_cfg->slave, CHIP_SELECT_COUNT);
		return -EINVAL;
	}

	master_handle.transferSize = word_size;

	master_config.channelConfig.polarity = (SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPOL) ?
						       kECSPI_PolarityActiveLow :
						       kECSPI_PolarityActiveHigh;

	master_config.channelConfig.phase = (SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPHA) ?
						    kECSPI_ClockPhaseSecondEdge :
						    kECSPI_ClockPhaseFirstEdge;

	if (spi_cfg->operation & SPI_TRANSFER_LSB) {
		LOG_ERR("Byte order must be MSB first");
		return -EINVAL;
	}

	master_config.baudRate_Bps = spi_cfg->frequency;

	master_config.chipSelectDelay = config->chip_select_delay;
	master_config.samplePeriod = config->sample_period;
	master_config.samplePeriodClock = config->sample_period_clock_source;
	master_config.burstLength = config->burst_length;

	master_config.channelConfig.clockInactiveState = config->clock_inactive_state;
	master_config.channelConfig.dataLineInactiveState = config->data_line_inactive_state;
	master_config.channelConfig.chipSlectActiveState = config->chip_select_active_state;

	if (clock_control_get_rate(config->clock_dev, config->clock_subsys, &clock_freq)) {
		return -EINVAL;
	}

	ECSPI_MasterInit(base, &master_config, clock_freq);

	ECSPI_MasterTransferCreateHandle(base, &data->handle, spi_mcux_master_transfer_callback,
					 data);

	data->ctx.config = spi_cfg;

	return 0;
}

static int transceive(const struct device *dev, const struct spi_config *spi_cfg,
		      const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs,
		      bool asynchronous, struct k_poll_signal *signal)
{
	struct spi_mcux_data *data = dev->data;
	int ret;

	spi_context_lock(&data->ctx, asynchronous, signal, spi_cfg);

	ret = spi_mcux_configure(dev, spi_cfg);
	if (ret) {
		goto out;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	spi_context_cs_control(&data->ctx, true);

	spi_mcux_transfer_next_packet(dev);

	ret = spi_context_wait_for_completion(&data->ctx);
out:
	spi_context_release(&data->ctx, ret);

	return ret;
}

static int spi_mcux_transceive(const struct device *dev, const struct spi_config *spi_cfg,
			       const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	return transceive(dev, spi_cfg, tx_bufs, rx_bufs, false, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_mcux_transceive_async(const struct device *dev, const struct spi_config *spi_cfg,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs, struct k_poll_signal *async)
{
	return transceive(dev, spi_cfg, tx_bufs, rx_bufs, true, async);
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_mcux_release(const struct device *dev, const struct spi_config *spi_cfg)
{
	struct spi_mcux_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static int spi_mcux_init(const struct device *dev)
{
	int err;
	const struct spi_mcux_config *config = dev->config;
	struct spi_mcux_data *data = dev->data;

	config->irq_config_func(dev);

	data->dev = dev;

	err = clock_control_on(config->clock_dev, config->clock_subsys);
	if (err) {
		LOG_ERR("Failed to enable clock (err %d)", err);
		return -EINVAL;
	}

	err = spi_context_cs_configure_all(&data->ctx);
	if (err < 0) {
		return err;
	}

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct spi_driver_api spi_mcux_driver_api = {
	.transceive = spi_mcux_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_mcux_transceive_async,
#endif
	.release = spi_mcux_release,
};

#define SPI_DMA_CHANNELS(n)

#define SPI_MCUX_ECSPI_INIT(n)                                                                     \
	static void spi_mcux_config_func_##n(const struct device *dev);                            \
                                                                                                   \
	static const struct spi_mcux_config spi_mcux_config_##n = {                                \
		.base = (ECSPI_Type *)DT_INST_REG_ADDR(n),                                         \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),              \
		.irq_config_func = spi_mcux_config_func_##n,                                       \
		.chip_select_delay = UTIL_AND(DT_INST_NODE_HAS_PROP(n, chip_select_delay),         \
					      DT_INST_PROP(n, chip_select_delay)),                 \
		.sample_period = UTIL_AND(DT_INST_NODE_HAS_PROP(n, sample_period),                 \
					  DT_INST_PROP(n, sample_period)),                         \
		.sample_period_clock_source = (ecspi_sample_period_clock_source_t)UTIL_AND(        \
			DT_INST_NODE_HAS_PROP(n, sample_period_clock_source),                      \
			DT_INST_PROP(n, sample_period_clock_source)),                              \
		.burst_length = UTIL_AND(DT_INST_NODE_HAS_PROP(n, burst_length),                   \
					 DT_INST_PROP(n, burst_length)),                           \
		.clock_inactive_state = (ecspi_clock_inactive_state_t)UTIL_AND(                    \
			DT_INST_NODE_HAS_PROP(n, clock_inactive_state),                            \
			DT_INST_PROP(n, clock_inactive_state)),                                    \
		.data_line_inactive_state = (ecspi_data_line_inactive_state_t)UTIL_AND(            \
			DT_INST_NODE_HAS_PROP(n, data_line_inactive_state),                        \
			DT_INST_PROP(n, data_line_inactive_state)),                                \
		.chip_select_active_state = (ecspi_chip_select_active_state_t)UTIL_AND(            \
			DT_INST_NODE_HAS_PROP(n, chip_select_active_state),                        \
			DT_INST_PROP(n, chip_select_active_state)),                                \
	};                                                                                         \
                                                                                                   \
	static struct spi_mcux_data spi_mcux_data_##n = {                                          \
		SPI_CONTEXT_INIT_LOCK(spi_mcux_data_##n, ctx),                                     \
		SPI_CONTEXT_INIT_SYNC(spi_mcux_data_##n, ctx),                                     \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx) SPI_DMA_CHANNELS(n)           \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &spi_mcux_init, NULL, &spi_mcux_data_##n, &spi_mcux_config_##n,   \
			      POST_KERNEL, CONFIG_SPI_INIT_PRIORITY, &spi_mcux_driver_api);        \
                                                                                                   \
	static void spi_mcux_config_func_##n(const struct device *dev)                             \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), spi_mcux_isr,               \
			    DEVICE_DT_INST_GET(n), 0);                                             \
                                                                                                   \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}

DT_INST_FOREACH_STATUS_OKAY(SPI_MCUX_ECSPI_INIT)
