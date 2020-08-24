// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for LTC6952 ultralow jitter, JESD204B/C clock generation IC.
 *
 * Copyright 2019 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/gcd.h>
#include <linux/rational.h>

#include <linux/clk.h>
#include <linux/clk-provider.h>


/* Register address macro */
#define LTC6952_REG(x)		(x)

/* LTC6952_REG0 */
#define LTC6952_UNLOCK_MSK	BIT(6)
#define LTC6952_LOCK_MSK	BIT(4)
#define LTC6952_VCOOK_MSK	BIT(2)
#define LTC6952_REFOK_MSK	BIT(0)

/* LTC6952_REG1 */
#define LTC6952_INVSTAT_MSK	BIT(7)
#define LTC6952_INVSTAT(x)	FIELD_PREP(LTC6952_INVSTAT_MSK, x)
#define LTC6952_STAT_OUT_MSK	GENMASK(6, 0)
#define LTC6952_STAT_OUT(x)	FIELD_PREP(LTC6952_STAT_OUT_MSK, x)

/* LTC6952_REG2 */
#define LTC6952_PDALL_MSK	BIT(7)
#define LTC6952_PDALL(x)	FIELD_PREP(LTC6952_PDALL_MSK, x)
#define LTC6952_PDPLL_MSK	BIT(6)
#define LTC6952_PDPLL(x)	FIELD_PREP(LTC6952_PDPLL_MSK, x)
#define LTC6952_PDVCOPK_MSK	BIT(5)
#define LTC6952_PDVCOPK(x)	FIELD_PREP(LTC6952_PDVCOPK_MSK, x)
#define LTC6952_PDREFPK_MSK	BIT(4)
#define LTC6952_PDREFPK(x)	FIELD_PREP(LTC6952_PDREFPK_MSK, x)
#define LTC6952_BST_MSK		BIT(3)
#define LTC6952_BST(x)		FIELD_PREP(LTC6952_BST_MSK, x)
#define LTC6952_FILTR_MSK	BIT(2)
#define LTC6952_FILTR(x)	FIELD_PREP(LTC6952_FILTR_MSK, x)
#define LTC6952_FILTV_MSK	BIT(1)
#define LTC6952_FILTV(x)	FIELD_PREP(LTC6952_FILTV_MSK, x)
#define LTC6952_POR_MSK		BIT(0)
#define LTC6952_POR(x)		FIELD_PREP(LTC6952_POR_MSK, x)

/* LTC6952_REG3 */
#define LTC6952_PD3_MSK		GENMASK(7, 6)
#define LTC6952_PD3(x)		FIELD_PREP(LTC6952_PD3_MSK, x)
#define LTC6952_PD2_MSK		GENMASK(5, 4)
#define LTC6952_PD2(x)		FIELD_PREP(LTC6952_PD2_MSK, x)
#define LTC6952_PD1_MSK		GENMASK(3, 2)
#define LTC6952_PD1(x)		FIELD_PREP(LTC6952_PD1_MSK, x)
#define LTC6952_PD0_MSK		GENMASK(1, 0)
#define LTC6952_PD0(x)		FIELD_PREP(LTC6952_PD0_MSK, x)

/* LTC6952_REG4 */
#define LTC6952_PD7_MSK		GENMASK(7, 6)
#define LTC6952_PD7(x)		FIELD_PREP(LTC6952_PD7_MSK, x)
#define LTC6952_PD6_MSK		GENMASK(5, 4)
#define LTC6952_PD6(x)		FIELD_PREP(LTC6952_PD6_MSK, x)
#define LTC6952_PD5_MSK		GENMASK(3, 2)
#define LTC6952_PD5(x)		FIELD_PREP(LTC6952_PD5_MSK, x)
#define LTC6952_PD4_MSK		GENMASK(1, 0)
#define LTC6952_PD4(x)		FIELD_PREP(LTC6952_PD4_MSK, x)

/* LTC6952_REG5 */
#define LTC6952_TEMPO_MSK	BIT(7)
#define LTC6952_TEMPO(x)	FIELD_PREP(LTC6952_TEMPO_MSK, x)
#define LTC6952_PD10_MSK	GENMASK(5, 4)
#define LTC6952_PD10(x)		FIELD_PREP(LTC6952_PD10_MSK, x)
#define LTC6952_PD9_MSK		GENMASK(3, 2)
#define LTC6952_PD9(x)		FIELD_PREP(LTC6952_PD9_MSK, x)
#define LTC6952_PD8_MSK		GENMASK(1, 0)
#define LTC6952_PD8(x)		FIELD_PREP(LTC6952_PD8_MSK, x)

#define LTC6952_PD_MSK(ch)	GENMASK(((ch) & 0x03) * 2 + 1, ((ch) & 0x03) * 2)
#define LTC6952_PD(ch, x)	((x) << ((ch) & 0x03))

/* LTC6952_REG6 */
#define LTC6952_RAO_MSK		BIT(7)
#define LTC6952_RAO(x)		FIELD_PREP(LTC6952_RAO_MSK, x)
#define LTC6952_PARSYNC_MSK	BIT(6)
#define LTC6952_PARSYNC(x)	FIELD_PREP(LTC6952_PARSYNC_MSK, x)
#define LTC6952_LKWIN_MSK	BIT(4)
#define LTC6952_LKWIN(x)	FIELD_PREP(LTC6952_LKWIN_MSK, x)
#define LTC6952_LKCT_MSK	GENMASK(3, 2)
#define LTC6952_LKCT(x)		FIELD_PREP(LTC6952_LKCT_MSK, x)
#define LTC6952_RD_HIGH_MSK	GENMASK(1, 0)
#define LTC6952_RD_HIGH(x)	FIELD_PREP(LTC6952_RD_HIGH_MSK, x)

/* LTC6952_REG10 */
#define LTC6952_CPRST_MSK	BIT(7)
#define LTC6952_CPRST(x)	FIELD_PREP(LTC6952_CPRST_MSK, x)
#define LTC6952_CPUP_MSK	BIT(6)
#define LTC6952_CPUP(x)		FIELD_PREP(LTC6952_CPUP_MSK, x)
#define LTC6952_CPDN_MSK	BIT(5)
#define LTC6952_CPDN(x)		FIELD_PREP(LTC6952_CPDN_MSK, x)
#define LTC6952_CP_MSK		GENMASK(4, 0)
#define LTC6952_CP(x)		FIELD_PREP(LTC6952_CP_MSK, x)

/* LTC6952_REG11 */
#define LTC6952_CPMID_MSK	BIT(7)
#define LTC6952_CPMID(x)	FIELD_PREP(LTC6952_CPMID_MSK, x)
#define LTC6952_CPWIDE_MSK	BIT(6)
#define LTC6952_CPWIDE(x)	FIELD_PREP(LTC6952_CPWIDE_MSK, x)
#define LTC6952_CPINV_MSK	BIT(5)
#define LTC6952_CPINV(x)	FIELD_PREP(LTC6952_CPINV_MSK, x)
#define LTC6952_EZMD_MSK	BIT(4)
#define LTC6952_EZMD(x)		FIELD_PREP(LTC6952_EZMD_MSK, x)
#define LTC6952_SRQMD_MSK	BIT(3)
#define LTC6952_SRQMD(x)	FIELD_PREP(LTC6952_SRQMD_MSK, x)
#define LTC6952_SYSCT_MSK	GENMASK(2, 1)
#define LTC6952_SYSCT(x)	FIELD_PREP(LTC6952_SYSCT_MSK, x)
#define LTC6952_SSRQ_MSK	BIT(0)
#define LTC6952_SSRQ(x)		FIELD_PREP(LTC6952_SSRQ_MSK, x)

/* LTC6952_REG12,16,20,24,28,32,36,40,44,48,52 */
#define LTC6952_MP_MSK		GENMASK(7, 3)
#define LTC6952_MP(x)		FIELD_PREP(LTC6952_MP_MSK, x)
#define LTC6952_MD_MSK		GENMASK(2, 0)
#define LTC6952_MD(x)		FIELD_PREP(LTC6952_MD_MSK, x)

/* LTC6952_REG13,17,21,25,29,33,37,41,45,49,53 */
#define LTC6952_SRQEN_MSK	BIT(7)
#define LTC6952_SRQEN(x)	FIELD_PREP(LTC6952_SRQEN_MSK, x)
#define LTC6952_MODE_MSK	GENMASK(6, 5)
#define LTC6952_MODE(x)		FIELD_PREP(LTC6952_MODE_MSK, x)
#define LTC6952_OINV_MSK	BIT(4)
#define LTC6952_OINV(x)		FIELD_PREP(LTC6952_OINV_MSK, x)
#define LTC6952_DDEL_HIGH_MSK	GENMASK(3, 0)
#define LTC6952_DDEL_HIGH(x)	FIELD_PREP(LTC6952_DDEL_HIGH_MSK, x)

/* LTC6952_REG15,19,23,27,31,35,39,43,47,51,55 */
#define LTC6952_ADEL_MSK	GENMASK(5, 0)
#define LTC6952_ADEL(x)		FIELD_PREP(LTC6952_ADEL_MSK, x)

/* LTC6952_REG56 */
#define LTC6952_REV_MSK		GENMASK(7, 4)
#define LTC6952_PART_MSK	GENMASK(3, 0)

#define LTC6952_CMD_READ	0x1
#define LTC6952_CMD_WRITE	0x0
#define LTC6952_CMD_ADDR(x)	((x) << 1)

#define LTC6952_NUM_CLKS	11

#define LTC6952_N_MAX		65535
#define LTC6952_R_MAX		1023

#define LTC6952_PFD_FREQ_MAX	167000

#define LTC6952_OUT_DIV_MIN	1
#define LTC6952_OUT_DIV_MAX	1048576

#define LTC6952_CP_CURRENT_MIN	423
#define LTC6952_CP_CURRENT_MAX	11200

#define LTC6952_CH_OFFSET(ch)	(0x04 * (ch))

struct ltc6952_clk_hw {
	unsigned int	address;
	struct clk_hw	hw;
	struct ltc6952_driver_data *drvdata;
};

struct ltc6952_clk_state {
	unsigned int		num;
	unsigned int		out_divider;
	unsigned int		mp;
	unsigned int		md;
	unsigned int		digital_delay;
	unsigned int		analog_delay;
	unsigned int		sysref_mode;
	unsigned int		power_down_mode;
	const char		*extended_name;
};

struct ltc6952_driver_data {
	struct spi_device		*spi;
	struct mutex			lock;
	struct clk			ref_clk;
	struct clk			vco_clk;
	struct clk			*clks[LTC6952_NUM_CLKS];
	struct clk_onecell_data		clk_data;
	const char			*clk_out_names[LTC6952_NUM_CLKS];
	bool				follower;
	u32				cp_current;
	struct ltc6952_clk_state	*clk_state;
	struct ltc6952_clk_hw		clks_hw[LTC6952_NUM_CLKS];
	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	union {
		__be32 d32;
		u8 d8[2];
	} data ____cacheline_aligned;
};

#define to_ltc6952_clk(_hw) container_of(_hw, struct ltc6952_clk_hw, hw)

static int ltc6952_write(struct ltc6952_state *st,
			 unsigned int reg,
			 u8 val)
{
	st->data.d[0] =	LTC6952_CMD_WRITE | LTC6952_CMD_ADDR(reg);
	st->data.d[1] = val;

	return spi_write(st->spi, st->data, ARRAY_SIZE(buf));
}

static int ltc6952_read(struct ltc6952_state *st,
			unsigned int reg,
			u8 *val)
{
	u8 cmd;

	cmd = LTC6952_CMD_READ | LTC6952_CMD_ADDR(reg);

	return spi_write_then_read(st->spi, &cmd, 1, val, 1);
}

static int ltc6952_update_bits(struct iio_dev *indio_dev,
				 unsigned int addr,
				 unsigned long mask,
				 unsigned int val)
{
	int readval, ret;

	ret = ltc6952_read(indio_dev, addr, &readval);
	if (ret < 0)
		return ret;

	readval &= ~mask;
	readval |= val;

	return ltc6952_write(indio_dev, addr, readval);
}

static unsigned int ltc6952_calc_out_div(unsigned long parent_rate,
					 unsigned long rate)
{
	unsigned int div;

	div = DIV_ROUND_CLOSEST(parent_rate, rate);

	div = clamp_t(unsigned int,
		      div,
		      LTC6952_OUT_DIV_MIN,
		      LTC6952_OUT_DIV_MAX);

	return div;
}

static int ltc6952_calculate_divider(struct ltc6952_clk_state *clk_st)
{
	int mp, md;
	unsigned int out_divider;

	/* M(x) = (MP(x) + 1)2^MD(x) */
	clk_st->md = 0;
	for (mp = 0; mp < 32; mp++) {
		if (clk_st->out_divider == (mp + 1)) {
			clk_st->mp = mp;
			clk_st->md = 0;
			return 0;
		}

		/* MD works only if MP is greater than 15 */
		if (mp <= 15)
			continue;
		for (md = 0; md < 7; md++) {
			out_divider = (mp + 1) << md;
			if (clk_st->out_divider == out_divider) {
				clk_st->mp = mp;
				clk_st->md = md;
				return 0;
			}
			if (clk_st->out_divider < out_divider)
				break;
		}
	}

	return -EINVAL;
}

static unsigned long ltc6952_clk_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct ltc6952_clk_hw *clk_hw = to_ltc6952_clk(hw);
	struct ltc6952_driver_data *drvdata = clk_hw->drvdata;
	unsigned int address = clk_hw->address;

	return drv_data->vco_freq / drvdata->clk_state[address];
}

static long ltc6952_clk_round_rate(struct clk_hw *hw,
				   unsigned long rate,
				   unsigned long *parent_rate)
{
	struct ltc6952_clk_hw *clk_hw = to_ltc6952_clk(hw);
	struct ltc6952_driver_data *drvdata = clk_hw->drvdata;
	unsigned int div;

	div = ltc6952_calc_out_div(drvdata->vco_freq, rate);

	return DIV_ROUND_CLOSEST(st->vco_freq, div);
}

static int ltc6952_clk_set_rate(struct clk_hw *hw,
				unsigned long rate,
				unsigned long parent_rate)
{
	struct ltc6952_clk_hw *clk_hw = to_ltc6952_clk(hw);
	struct ltc6952_driver_data *drvdata = clk_hw->drvdata;
	unsigned int address = clk_hw->address;
	struct ltc6952_clk_state *clk_st = drvdata->clk_state[address];
	int ret;

	clk_st->out_divider = ltc6952_calc_out_div(drvdata->vco_freq, rate);
	ret = ltc6952_calculate_divider(ch);
	if (ret < 0)
		return ret;

	mutex_lock(&drvdata->lock);
	tmp = LTC6952_MP(clk_st->mp) | LTC6952_MD(clk_st->md);
	ret = ltc6952_write(drvdata->spi, LTC6952_REG(0x0C) +
			    LTC6952_CH_OFFSET(clk_st->num),
			    tmp);
	mutex_unlock(&drvdata->lock);

	return ret;
}

static int ltc6952_clk_get_phase(struct clk_hw *hw)
{

}

static int ltc6952_clk_set_phase(struct clk_hw *hw, int degrees)
{

}

static const struct clk_ops ltc6952_clk_ops = {
	.prepare = ltc6952_clk_prepare,
	.unprepare = ltc6952_clk_unprepare,
	.recalc_rate = ltc6952_clk_recalc_rate,
	.round_rate = ltc6952_clk_round_rate,
	.set_rate = ltc6952_clk_set_rate,
	.set_phase = ltc6952_clk_set_phase,
	.get_phase = ltc6952_clk_get_phase,
};

static int ltc6952_parse_dt(struct device *dev,
			    struct ltc6952_state *st)
{
	struct device_node *np = dev->of_node, *chan_np;
	unsigned int cnt = 0;
	int ret;

	ret = of_property_read_u32(np, "adi,ref-frequency-hz",
			     &st->ref_freq);
	if (ret < 0)
		return ret;

	ret = of_property_read_u32(np, "adi,vco-frequency-hz",
				   &st->vco_freq);
	if (ret < 0)
		return ret;

	st->follower = of_property_read_bool(np, "adi,follower-mode-enable");

	ret = of_property_read_u32(np, "adi,charge-pump-microamp",
				   &st->cp_current);
	if (ret < 0)
		st->cp_current = LTC6952_CP_CURRENT_MAX;
	else
		st->cp_current = clamp_t(u32, st->cp_current,
			LTC6952_CP_CURRENT_MIN, LTC6952_CP_CURRENT_MAX);

	ret = of_property_read_string_array(np, "clock-output-names",
			st->clk_out_names, ARRAY_SIZE(st->clk_out_names));
	if (ret < 0)
		return ret;

	st->num_channels = of_get_available_child_count(np);
	if (st->num_channels > LTC6952_NUM_CHAN)
		return -EINVAL;

	st->channels = devm_kzalloc(dev,
		sizeof(struct ltc6952_chan_spec) * st->num_channels,
				     GFP_KERNEL);
	if (!st->channels)
		return -ENOMEM;

	for_each_child_of_node(np, chan_np) {
		st->channels[cnt].num = cnt;
		of_property_read_u32(chan_np, "reg",
				     &st->channels[cnt].num);

		if (of_property_read_u32(chan_np, "adi,divider",
					 &st->channels[cnt].out_divider))
			st->channels[cnt].out_divider = 4;

		of_property_read_u32(chan_np, "adi,digital-delay",
				     &st->channels[cnt].digital_delay);

		of_property_read_u32(chan_np, "adi,analog-delay",
				     &st->channels[cnt].analog_delay);

		of_property_read_string(chan_np, "adi,extended-name",
					&st->channels[cnt].extended_name);

		cnt++;
	}

	return 0;
}

static int ltc6952_setup(struct spi_device *spi)
{
	struct ltc6952_state *st = iio_priv(indio_dev);
	struct ltc6952_chan_spec *chan;
	unsigned long vco_freq, ref_freq;
	unsigned long pfd_freq;
	unsigned long n, r;
	unsigned int i, tmp;
	u32 cp_current;
	int ret;

	mutex_lock(&st->lock);
	/* Resets all registers to default values */
	ret = ltc6952_write_mask(indio_dev, LTC6952_REG(0x02),
				 LTC6952_POR_MSK, LTC6952_POR(1));
	if (ret < 0)
		goto err_unlock;

	if (st->follower) {
		ret = ltc6952_write_mask(indio_dev, LTC6952_REG(0x02),
					LTC6952_PDPLL_MSK, LTC6952_PDPLL(1));
		if (ret < 0)
			goto err_unlock;
		goto follower;
	}

	vco_freq = st->vco_freq / 1000;
	ref_freq = st->ref_freq / 1000;

	/* fVCO / N = fREF / R */
	rational_best_approximation(vco_freq, ref_freq,
				    LTC6952_N_MAX, LTC6952_R_MAX,
				    &n, &r);

	pfd_freq = vco_freq / n;
	while ((pfd_freq > LTC6952_PFD_FREQ_MAX) &&
	       (n <= LTC6952_N_MAX / 2) &&
	       (r <= LTC6952_R_MAX / 2)) {
		pfd_freq /= 2;
		n *= 2;
		r *= 2;
	}

	/* Program the dividers */
	ret |= ltc6952_write_mask(indio_dev, LTC6952_REG(0x06),
				  LTC6952_RD_HIGH_MSK,
				  LTC6952_RD_HIGH((r & 0x300) >> 8));
	ret |= ltc6952_write(indio_dev, LTC6952_REG(0x07), LTC6952_RD_LOW(r));
	ret |= ltc6952_write(indio_dev, LTC6952_REG(0x08),
			     LTC6952_ND_HIGH(n >> 8));
	ret |= ltc6952_write(indio_dev, LTC6952_REG(0x09), LTC6952_ND_LOW(n));
	if (ret < 0)
		goto err_unlock;

	/* PLL lock cycle count  */
	ret = ltc6952_write_mask(indio_dev, LTC6952_REG(0x06), LTC6952_LKCT_MSK,
				 LTC6952_LKCT(0x03));
	if (ret < 0)
		goto err_unlock;

	/* Disable CP Hi-Z and set the CP current */
	cp_current = st->cp_current / LTC6952_CP_CURRENT_MIN;
	ret = ltc6952_write_mask(indio_dev, LTC6952_REG(0x0A),
				 LTC6952_CPRST_MSK | LTC6952_CP_MSK,
				 LTC6952_CPRST(0x0) | LTC6952_CP(cp_current));
	if (ret < 0)
		goto err_unlock;

follower:
	/* Program the output channels */
	for (i = 0; i < st->num_channels; i++) {
		chan = &st->channels[i];

		if (chan->num >= LTC6952_NUM_CHAN)
			continue;

		/* Enable channel */
		tmp = LTC6952_PD(i, 0);
		ret |= ltc6952_write_mask(indio_dev,
					  LTC6952_REG(0x03) + (chan->num >> 2),
					  LTC6952_PD_MSK(chan->num), tmp);

		ret |= ltc6952_calculate_divider(chan);
		ret |= ltc6952_write(indio_dev, LTC6952_REG(0x0c) +
				     LTC6952_CH_OFFSET(chan->num),
				     LTC6952_MP(chan->mp) | LTC6952_MD(chan->md));

		/* Enable sync or sysref */
		ret |= ltc6952_write_mask(indio_dev, LTC6952_REG(0x0D) +
					  LTC6952_CH_OFFSET(chan->num),
					  LTC6952_SRQEN_MSK,
					  LTC6952_SRQEN(1));

		/* Set channel delay */
		ret |= ltc6952_write_mask(indio_dev, LTC6952_REG(0x0D) +
					  LTC6952_CH_OFFSET(chan->num),
					  LTC6952_DDEL_HIGH_MSK,
					  LTC6952_DDEL_HIGH((chan->digital_delay &
							     0xF00) >> 8));
		ret |= ltc6952_write(indio_dev, LTC6952_REG(0x0E) +
				     LTC6952_CH_OFFSET(chan->num),
				     LTC6952_DDEL_LOW(chan->digital_delay));
		ret |= ltc6952_write(indio_dev, LTC6952_REG(0x0F) +
				     LTC6952_CH_OFFSET(chan->num),
				     LTC6952_ADEL(chan->analog_delay));
		if (ret < 0)
			goto err_unlock;

		st->iio_channels[i].type = IIO_ALTVOLTAGE;
		st->iio_channels[i].output = 1;
		st->iio_channels[i].indexed = 1;
		st->iio_channels[i].channel = chan->num;
		st->iio_channels[i].address = i;
		st->iio_channels[i].extend_name = chan->extended_name;
		st->iio_channels[i].info_mask_separate =
			BIT(IIO_CHAN_INFO_FREQUENCY) |
			BIT(IIO_CHAN_INFO_PHASE);
	}

	/* Phase Syncronization */
	ret |= ltc6952_write_mask(indio_dev, LTC6952_REG(0x0B),
				  LTC6952_SSRQ_MSK, LTC6952_SSRQ(1));
	usleep_range(1000, 1001); /* sleep > 1000us */
	ret |= ltc6952_write_mask(indio_dev, LTC6952_REG(0x0B),
				  LTC6952_SSRQ_MSK, LTC6952_SSRQ(0));
	if (ret < 0)
		goto err_unlock;

	mutex_unlock(&st->lock);

	/* Configure clocks */
	for (i = 0; i < st->num_channels; i++) {
		chan = &st->channels[i];

		if (chan->num >= LTC6952_NUM_CHAN)
			continue;

		ret = ltc6952_clk_register(indio_dev, chan->num, i);
		if (ret)
			return ret;
	}

	st->clk_data.clks = st->clks;
	st->clk_data.clk_num = LTC6952_NUM_CHAN;

	return of_clk_add_provider(st->spi->dev.of_node,
				   of_clk_src_onecell_get,
				   &st->clk_data);
err_unlock:
	mutex_unlock(&st->lock);
	return ret;
}

static const struct spi_device_id ltc6952_id[] = {
	{"ltc6952", 0},
	{}
};
MODULE_DEVICE_TABLE(spi, ltc6952_id);

static const struct of_device_id ltc6952_of_match[] = {
	{ .compatible = "adi,ltc6952" },
	{ },
};
MODULE_DEVICE_TABLE(of, ltc6952_of_match);

static struct spi_driver ltc6952_driver = {
	.driver = {
		.name = "ltc6952",
		.of_match_table = ltc6952_of_match,
	},
	.probe = ltc6952_probe,
	.remove = ltc6952_remove,
	.id_table = ltc6952_id,
};
module_spi_driver(ltc6952_driver);

MODULE_AUTHOR("Mircea Caprioru <mircea.caprioru@analog.com>");
MODULE_DESCRIPTION("Analog Devices LTC6952 driver");
MODULE_LICENSE("GPL v2");
