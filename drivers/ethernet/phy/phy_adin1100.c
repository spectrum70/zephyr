/*
 * Copyright (c) 2024 BayLibre, SAS
 * Copyright (c) 2010-2024 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_adin1100_phy

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/net/phy.h>
#include <zephyr/net/mii.h>
#include <zephyr/net/mdio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(phy_mii, CONFIG_PHY_LOG_LEVEL);

#define MII_INVALID_PHY_ID			UINT32_MAX

/* ADIN1100 PHY identifier */
#define ADIN1100_PHY_ID				0x0283BC81U

/* LED Control Register */
#define ADIN1100_PHY_LED_CNTRL			0x8C82U
/* LED 1 Enable */
#define ADIN1100_PHY_LED_CNTRL_LED1_EN		BIT(15)
/* LED 0 Enable */
#define ADIN1100_PHY_LED_CNTRL_LED0_EN		BIT(7)

/* MMD bridge regs */
#define ADIN1100_MMD_ACCESS_CNTRL		0x0DU
#define ADIN1100_MMD_ACCESS			0x0EU

struct phy_adin1100_config {
	const struct device *mdio;
	uint8_t phy_addr;
	bool led0_en;
	bool led1_en;
	bool tx_24v;
};

struct phy_adin1100_data {
	const struct device *dev;
	struct phy_link_state state;
	struct k_sem sem;
	struct k_work_delayable monitor_work;
	phy_callback_t cb;
	void *cb_data;
};

static inline int phy_adin1100_c22_read(const struct device *dev, uint16_t reg,
					uint16_t *val)
{
	const struct phy_adin1100_config *const cfg = dev->config;

	return mdio_read(cfg->mdio, cfg->phy_addr, reg, val);
}

static inline int phy_adin1100_c22_write(const struct device *dev, uint16_t reg,
					 uint16_t val)
{
	const struct phy_adin1100_config *const cfg = dev->config;

	return mdio_write(cfg->mdio, cfg->phy_addr, reg, val);
}

static inline int phy_adin1100_c45_setup_dev_reg(const struct device *dev, uint16_t devad,
						 uint16_t reg)
{
	const struct phy_adin1100_config *cfg = dev->config;
	int rval;

	rval = mdio_write(cfg->mdio, cfg->phy_addr, ADIN1100_MMD_ACCESS_CNTRL, devad);
	if (rval < 0) {
		return rval;
	}
	rval = mdio_write(cfg->mdio, cfg->phy_addr, ADIN1100_MMD_ACCESS, reg);
	if (rval < 0) {
		return rval;
	}

	return mdio_write(cfg->mdio, cfg->phy_addr, ADIN1100_MMD_ACCESS_CNTRL, devad | BIT(14));
}

static inline int phy_adin1100_c45_read(const struct device *dev, uint16_t devad,
					uint16_t reg, uint16_t *val)
{
	const struct phy_adin1100_config *cfg = dev->config;
	int rval;

	/* Using C22 -> devad bridge */
	rval = phy_adin1100_c45_setup_dev_reg(dev, devad, reg);
	if (rval < 0) {
		return rval;
	}

	return mdio_read(cfg->mdio, cfg->phy_addr, ADIN1100_MMD_ACCESS, val);
}

static inline int phy_adin1100_c45_write(const struct device *dev, uint16_t devad,
					 uint16_t reg, uint16_t val)
{
	const struct phy_adin1100_config *cfg = dev->config;
	int rval;

	/* Using C22 -> devad bridge */
	rval = phy_adin1100_c45_setup_dev_reg(dev, devad, reg);
	if (rval < 0) {
		return rval;
	}

	return mdio_write(cfg->mdio, cfg->phy_addr, ADIN1100_MMD_ACCESS, val);
}

static int phy_adin1100_reg_read(const struct device *dev, uint16_t reg_addr,
				 uint32_t *data)
{
	const struct phy_adin1100_config *cfg = dev->config;
	int ret;

	mdio_bus_enable(cfg->mdio);

	ret = phy_adin1100_c22_read(dev, reg_addr, (uint16_t *) data);

	mdio_bus_disable(cfg->mdio);

	return ret;
}

static int phy_adin1100_reg_write(const struct device *dev, uint16_t reg_addr,
				  uint32_t data)
{
	const struct phy_adin1100_config *cfg = dev->config;
	int ret;

	mdio_bus_enable(cfg->mdio);

	ret = phy_adin1100_c22_write(dev, reg_addr, (uint16_t) data);

	mdio_bus_disable(cfg->mdio);

	return ret;
}

static int phy_adin1100_cfg_link(const struct device *dev, enum phy_link_speed adv_speeds)
{
	ARG_UNUSED(dev);

	if (!!(adv_speeds & LINK_FULL_10BASE_T)) {
		return 0;
	}

	return -ENOTSUP;
}

static int phy_adin1100_get_link_state(const struct device *dev,
				       struct phy_link_state *state)
{
	struct phy_adin1100_data *const data = dev->data;

	k_sem_take(&data->sem, K_FOREVER);

	memcpy(state, &data->state, sizeof(struct phy_link_state));

	k_sem_give(&data->sem);

	return 0;
}

static void invoke_link_cb(const struct device *dev)
{
	struct phy_adin1100_data *const data = dev->data;
	struct phy_link_state state;

	if (data->cb == NULL) {
		return;
	}

	/* Send callback only on link state change */
	if (phy_adin1100_get_link_state(dev, &state) != 0) {
		return;
	}

	data->cb(dev, &state, data->cb_data);
}

static int phy_adin1100_link_cb_set(const struct device *dev, phy_callback_t cb, void *user_data)
{
	struct phy_adin1100_data *const data = dev->data;

	data->cb = cb;
	data->cb_data = user_data;

	/* Invoke the callback to notify the caller of the current
	 * link status.
	 */
	invoke_link_cb(dev);

	return -ENOTSUP;
}

static int update_link_state(const struct device *dev)
{
	struct phy_adin1100_data *const data = dev->data;
	const struct phy_adin1100_config *config = dev->config;
	struct phy_link_state old_state;
	uint16_t bmsr;
	int ret;

	ret = phy_adin1100_c22_read(dev, MII_BMSR, &bmsr);
	if (ret < 0) {
		return ret;
	}

	old_state = data->state;
	data->state.is_up = !!(bmsr & MII_BMSR_LINK_STATUS);

	if (memcmp(&old_state, &data->state, sizeof(struct phy_link_state)) != 0) {

		LOG_INF("PHY (%d) Link is %s", config->phy_addr, data->state.is_up ? "up" : "down");

		if (data->state.is_up == false) {
			return 0;
		}

		LOG_INF("PHY (%d) Link speed %s Mb, %s duplex\n", config->phy_addr,
			(PHY_LINK_IS_SPEED_100M(data->state.speed) ? "100" : "10"),
			PHY_LINK_IS_FULL_DUPLEX(data->state.speed) ? "full" : "half");
	}

	return 0;
}

static void monitor_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct phy_adin1100_data *const data =
		CONTAINER_OF(dwork, struct phy_adin1100_data, monitor_work);
	const struct device *dev = data->dev;
	int rc;

	k_sem_take(&data->sem, K_FOREVER);

	rc = update_link_state(dev);

	k_sem_give(&data->sem);

	/* If link state has changed and a callback is set, invoke callback */
	if (rc == 0) {
		invoke_link_cb(dev);
	}

	/* Submit delayed work */
	k_work_reschedule(&data->monitor_work, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
}

static int get_id(const struct device *dev, uint32_t *phy_id)
{
	uint16_t value;

	if (phy_adin1100_c22_read(dev, MII_PHYID1R, &value) < 0) {
		return -EIO;
	}

	*phy_id = value << 16;

	if (phy_adin1100_c22_read(dev, MII_PHYID2R, &value) < 0) {
		return -EIO;
	}

	*phy_id |= value;

	return 0;
}

static int phy_adin1100_init(const struct device *dev)
{
	struct phy_adin1100_data *const data = dev->data;
	const struct phy_adin1100_config *const cfg = dev->config;
	uint32_t phy_id;
	uint16_t val;
	bool tx_24v_supported = false;
	int ret;

	data->dev = dev;
	data->cb = NULL;
	data->state.is_up = false;
	/*
	 * Speed is fixed for this chip
	 * The mii is only able to operate in full-duplex
	 */
	data->state.speed = LINK_FULL_10BASE_T;

	if (get_id(dev, &phy_id) == 0) {
		if (phy_id == MII_INVALID_PHY_ID) {
			LOG_ERR("No PHY found at address %d",
				cfg->phy_addr);

			return -EINVAL;
		}

		if (phy_id != ADIN1100_PHY_ID) {
			LOG_ERR("PHY (%u) unexpected PHY ID %X", cfg->phy_addr, phy_id);
			return -EINVAL;
		}

		LOG_INF("PHY (%d) ID %X\n", cfg->phy_addr, phy_id);
	}

	if (!cfg->led0_en || !cfg->led1_en) {
		ret = phy_adin1100_c45_read(dev, MDIO_MMD_VENDOR_SPECIFIC1,
					    ADIN1100_PHY_LED_CNTRL, &val);
		if (ret < 0) {
			return ret;
		}
		if (!cfg->led0_en) {
			val &= ~(ADIN1100_PHY_LED_CNTRL_LED0_EN);
		}
		if (!cfg->led1_en) {
			val &= ~(ADIN1100_PHY_LED_CNTRL_LED1_EN);
		}
		ret = phy_adin1100_c45_write(dev, MDIO_MMD_VENDOR_SPECIFIC1,
					     ADIN1100_PHY_LED_CNTRL, val);
		if (ret < 0) {
			return ret;
		}
	}

	/* check 2.4V support */
	ret = phy_adin1100_c45_read(dev, MDIO_MMD_PMAPMD, MDIO_PMA_B10L_STAT, &val);
	if (ret < 0) {
		return ret;
	}

	tx_24v_supported = !!(val & MDIO_PMA_B10L_STAT_2V4_ABLE);

	LOG_INF("PHY (%u) 2.4V mode %s", cfg->phy_addr,
		tx_24v_supported ? "supported" : "not supported");

	if (tx_24v_supported) {
		val |= MDIO_AN_T1_ADV_H_10L_TX_HI;
	} else {
		val &= ~MDIO_AN_T1_ADV_H_10L_TX_HI;
	}

	if (cfg->tx_24v) {
		if (!tx_24v_supported) {
			LOG_ERR("PHY (%u) 2.4V mode enabled, but not supported",
				cfg->phy_addr);
			return -EINVAL;
		}
		LOG_INF("PHY (%u) 2.4V mode enabled", cfg->phy_addr);
		val |= MDIO_AN_T1_ADV_H_10L_TX_HI_REQ;
	} else {
		LOG_INF("PHY (%u) 2.4V mode disabled", cfg->phy_addr);
		val &= ~MDIO_AN_T1_ADV_H_10L_TX_HI_REQ;
	}

	ret = phy_adin1100_c45_write(dev, MDIO_MMD_AN, MDIO_AN_T1_ADV_H, val);
	if (ret < 0) {
		return ret;
	}

	/* enable auto-negotiation */
	ret = phy_adin1100_c45_write(dev, MDIO_MMD_AN, MDIO_AN_T1_CTRL,
				     MDIO_AN_T1_CTRL_EN);
	if (ret < 0) {
		return ret;
	}

	k_work_init_delayable(&data->monitor_work, monitor_work_handler);
	monitor_work_handler(&data->monitor_work.work);

	return 0;
}

static const struct ethphy_driver_api phy_adin1100_api = {
	.get_link = phy_adin1100_get_link_state,
	.cfg_link = phy_adin1100_cfg_link,
	.link_cb_set = phy_adin1100_link_cb_set,
	.read = phy_adin1100_reg_read,
	.write = phy_adin1100_reg_write,
};

#define ADIN1100_PHY_INITIALIZE(n)						\
	static const struct phy_adin1100_config phy_adin1100_config_##n = {	\
		.mdio = DEVICE_DT_GET(DT_INST_BUS(n)),				\
		.phy_addr = DT_INST_REG_ADDR(n),				\
		.led0_en = DT_INST_PROP(n, led0_en),				\
		.led1_en = DT_INST_PROP(n, led1_en),				\
	};									\
	static struct phy_adin1100_data phy_adin1100_data_##n = {		\
		.sem = Z_SEM_INITIALIZER(phy_adin1100_data_##n.sem, 1, 1),	\
	};									\
DEVICE_DT_INST_DEFINE(n, &phy_adin1100_init, NULL,				\
	&phy_adin1100_data_##n, &phy_adin1100_config_##n,			\
	POST_KERNEL, CONFIG_PHY_INIT_PRIORITY,					\
	&phy_adin1100_api);

DT_INST_FOREACH_STATUS_OKAY(ADIN1100_PHY_INITIALIZE)
