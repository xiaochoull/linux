// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices ONE_BIT_ADC_DAC
 * Digital to Analog Converters driver
 *
 * Copyright 2019 Analog Devices Inc.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>

enum {
	CH_IN,
	CH_OUT,
};

struct one_bit_adc_dac_state {
	struct platform_device 	*pdev;
	struct gpio_descs	*in_gpio_descs;
	struct gpio_descs	*out_gpio_descs;
};

 #define ONE_BIT_ADC_DAC_CHANNEL(idx, direction)			\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.channel = idx,						\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.output = direction,					\
	}

static int one_bit_adc_dac_read_raw(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int *val, int *val2, long info)
{
	struct one_bit_adc_dac_state *st = iio_priv(indio_dev);
	int channel = chan->channel;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if (channel < st->in_gpio_descs->ndescs) {
			*val = gpiod_get_value_cansleep(
				st->in_gpio_descs->desc[channel]);
		}
		else {
			channel -= st->in_gpio_descs->ndescs;
			*val = gpiod_get_value_cansleep(
				st->out_gpio_descs->desc[channel]);
		}
		printk("one_bit_adc_dac_read_raw val: %d\n", *val);
 		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int one_bit_adc_dac_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long info)
{
	struct one_bit_adc_dac_state *st = iio_priv(indio_dev);
	int channel = chan->channel;

	printk("one_bit_adc_dac_write_raw val: %d val2: %d\n", val, val2);
	
	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if (channel < st->in_gpio_descs->ndescs) {
			gpiod_set_value_cansleep(
				st->in_gpio_descs->desc[channel], val);
		}
		else {
			channel -= st->in_gpio_descs->ndescs;
			gpiod_set_value_cansleep(
				st->out_gpio_descs->desc[channel], val);
		}

		return 0;
	default:
		return -EINVAL;
	}
}

static const struct iio_info one_bit_adc_dac_info = {
	.read_raw = &one_bit_adc_dac_read_raw,
	.write_raw = &one_bit_adc_dac_write_raw,
};

static int one_bit_adc_dac_parse_dt(struct iio_dev *indio_dev)
{
	struct one_bit_adc_dac_state *st = iio_priv(indio_dev);
	static struct iio_chan_spec *channels;
	const char **out_gpio_names;
	const char **in_gpio_names;
	int ret, i, offset, num_channels;
	struct iio_chan_spec chan_spec;
	
	st->in_gpio_descs = devm_gpiod_get_array_optional(&st->pdev->dev, 
						"in", GPIOD_IN);
	if (IS_ERR(st->in_gpio_descs))
		return PTR_ERR(st->in_gpio_descs);

	st->out_gpio_descs = devm_gpiod_get_array_optional(&st->pdev->dev, 
						"out", GPIOD_OUT_HIGH);
	if (IS_ERR(st->out_gpio_descs))
		return PTR_ERR(st->out_gpio_descs);


	in_gpio_names = kcalloc(st->in_gpio_descs->ndescs, 
				sizeof(char*), GFP_KERNEL);
	if (!in_gpio_names)
		return -ENOMEM;

	out_gpio_names = kcalloc(st->out_gpio_descs->ndescs,
				sizeof(char*), GFP_KERNEL);
	if (!out_gpio_names)
		return -ENOMEM;

	num_channels = st->in_gpio_descs->ndescs + st->out_gpio_descs->ndescs;
	channels = kcalloc(num_channels,
				sizeof(struct iio_chan_spec), GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	ret = device_property_read_string_array(&st->pdev->dev,
					"input-gpio-names",
					in_gpio_names,
					st->in_gpio_descs->ndescs);
	if (ret < 0)
		return ret;

	ret = device_property_read_string_array(&st->pdev->dev,
					"out-gpio-names",
					out_gpio_names,
					st->out_gpio_descs->ndescs);
	if (ret < 0)
		return ret;

	for (i = 0; i < st->in_gpio_descs->ndescs; i++) {
		chan_spec = (struct iio_chan_spec)ONE_BIT_ADC_DAC_CHANNEL(i,
                                                                CH_IN);
		channels[i] = chan_spec;
		channels[i].extend_name = in_gpio_names[i];
	}

	for (i = 0; i < st->out_gpio_descs->ndescs; i++) {
		offset = st->in_gpio_descs->ndescs;
		chan_spec = (struct iio_chan_spec)ONE_BIT_ADC_DAC_CHANNEL(offset
                                                        + i, CH_OUT);
		channels[offset + i] = chan_spec;
		channels[offset + i].extend_name = out_gpio_names[i];
	}
	kfree(in_gpio_names);
	kfree(out_gpio_names);
	indio_dev->channels = channels;
	indio_dev->num_channels = num_channels;

	return 0;
}

static int one_bit_adc_dac_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct one_bit_adc_dac_state *st;
	int ret;

	printk("one-bit-adc-dac-teodorex\n");
 	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->pdev = pdev;
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = "one-bit-adc-dac";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &one_bit_adc_dac_info;

	ret = one_bit_adc_dac_parse_dt(indio_dev);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, indio_dev);
	return iio_device_register(indio_dev);
}

static int one_bit_adc_dac_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	iio_device_unregister(indio_dev);
	kfree(indio_dev->channels);

	return 0;
}

static const struct of_device_id one_bit_adc_dac_dt_match[] = {
	{ .compatible = "adi,one-bit-adc-dac" },
	{},
};

MODULE_DEVICE_TABLE(of, one_bit_adc_dac_dt_match);

static struct platform_driver one_bit_adc_dac_driver = {
	.driver = {
		.name = "one-bit-adc-dac",
		.of_match_table = one_bit_adc_dac_dt_match,
	},
	.probe = one_bit_adc_dac_probe,
	.remove = one_bit_adc_dac_remove,
};

module_platform_driver(one_bit_adc_dac_driver);

MODULE_AUTHOR("Cristian Pop <cristian.pop@analog.com>");
MODULE_DESCRIPTION("Analog Devices ONE_BIT_ADC_DAC");
MODULE_LICENSE("GPL v2");
