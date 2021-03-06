/* pinmux.c - general pinmux operation */

/*
 * Copyright (c) 2015 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <nanokernel.h>
#include <device.h>
#include <init.h>
#include <pinmux.h>
#include <sys_io.h>
#include "pinmux/pinmux.h"


#ifndef CONFIG_PINMUX_DEV
#define PRINT(...) {; }
#else
#if defined(CONFIG_PRINTK)
#include <misc/printk.h>
#define PRINT printk
#elif defined(CONFIG_STDOUT_CONSOLE)
#define PRINT printf
#endif /* CONFIG_PRINTK */
#endif /*CONFIG_PINMUX_DEV */

#define MASK_2_BITS             0x3
#define PINMUX_PULLUP_OFFSET	0x00
#define PINMUX_SLEW_OFFSET	0x10
#define PINMUX_INPUT_OFFSET	0x20
#define PINMUX_SELECT_OFFSET	0x30

#define PINMUX_SELECT_REGISTER(base, reg_offset) \
	(base + PINMUX_SELECT_OFFSET + (reg_offset << 2))

/*
 * A little decyphering of what is going on here:
 *
 * Each pinmux register rperesents a bank of 16 pins, 2 bits per pin for a total
 * of four possible settings per pin.
 *
 * The first argument to the macro is name of the uint32_t's that is being used
 * to contain the bit patterns for all the configuration registers.  The pin
 * number divided by 16 selects the correct register bank based on the pin
 * number.
 *
 * The pin number % 16 * 2 selects the position within the register bank for the
 * bits controlling the pin.
 *
 * All but the lower two bits of the config values are masked off to ensure
 * that we don't inadvertently affect other pins in the register bank.
 */
#define PIN_CONFIG(A, _pin, _func) \
	(A[((_pin) / 16)] |= ((0x3 & (_func)) << (((_pin) % 16) * 2)))

/*
 * This is the full pinmap that we have available on the board for configuration
 * including the ball position and the various modes that can be set.  In the
 * _pinmux_defaults we do not spend any time setting values that are using mode
 * A as the hardware brings up all devices by default in mode A.
 */

/* pin, ball, mode A, mode B, mode C */
/* 0  F02, gpio_0, ain_0, spi_s_cs */
/* 1  G04, gpio_1, ain_1, spi_s_miso */
/* 2  H05, gpio_2, ain_2, spi_s_sck */
/* 3  J06, gpio_3, ain_3, spi_s_mosi */
/* 4  K06, gpio_4, ain_4, NA */			/* 15.4 GPIO */
/* 5  L06, gpio_5, ain_5, NA */			/* 15.4 GPIO */
/* 6  H04, gpio_6, ain_6, NA */			/* 15.4 GPIO */
/* 7  G03, gpio_7, ain_7, NA */
/* 8  L05, gpio_ss_0, ain_8, uart1_cts */	/* UART debug */
/* 9  M05, gpio_ss_1, ain_9, uart1_rts */	/* UART debug */
/* 10 K05, gpio_ss_2, ain_10 */
/* 11 G01, gpio_ss_3, ain_11 */
/* 12 J04, gpio_ss_4, ain_12 */
/* 13 G02, gpio_ss_5, ain_13 */
/* 14 F01, gpio_ss_6, ain_14 */
/* 15 J05, gpio_ss_7, ain_15 */
/* 16 L04, gpio_ss_8, ain_16, uart1_txd */	/* UART debug */
/* 17 M04, gpio_ss_9, ain_17, uart1_rxd */	/* UART debug */
/* 18 K04, uart0_rx, ain_18, NA */		/* BT UART */
/* 19 B02, uart0_tx, gpio_31, NA */		/* BT UART */
/* 20 C01, i2c0_scl, NA, NA */			/* EEPROM, BT, Light Sensor */
/* 21 C02, i2c0_sda, NA, NA */			/* EEPROM, BT, Light Sensor */
/* 22 D01, i2c1_scl, NA, NA */
/* 23 D02, i2c1_sda, NA, NA */
/* 24 E01, i2c0_ss_sda, NA, NA */
/* 25 E02, i2c0_ss_scl, NA, NA */
/* 26 B03, i2c1_ss_sda, NA, NA */		/* IMU */
/* 27 A03, i2c1_ss_scl, NA, NA */		/* IMU */
/* 28 C03, spi0_ss_miso, NA, NA */		/* IMU */
/* 29 E03, spi0_ss_mosi, NA, NA */		/* IMU */
/* 30 D03, spi0_ss_sck, NA, NA */		/* IMU */
/* 31 D04, spi0_ss_cs0, NA, NA */		/* IMU */
/* 32 C04, spi0_ss_cs1, NA, NA */
/* 33 B04, spi0_ss_cs2, gpio_29, NA */		/* 15.4 GPIO */
/* 34 A04, spi0_ss_cs3, gpio_30, NA */
/* 35 B05, spi1_ss_miso, NA, NA */
/* 36 C05, spi1_ss_mosi, NA, NA */
/* 37 D05, spi1_ss_sck, NA, NA */
/* 38 E05, spi1_ss_cs0, NA, NA */
/* 39 E04, spi1_ss_cs1, NA, NA */
/* 40 A06, spi1_ss_cs2, uart0_cts, NA */	/* BT UART */
/* 41 B06, spi1_ss_cs3, uart0_rts, NA */	/* BT UART */
/* 42 C06, gpio_8, spi1_m_sck, NA */		/* 15.4 SPI */
/* 43 D06, gpio_9, spi1_m_miso, NA */		/* 15.4 SPI */
/* 44 E06, gpio_10, spi1_m_mosi, NA */		/* 15.4 SPI */
/* 45 D07, gpio_11, spi1_m_cs0, NA */		/* 15.4 SPI GPIO CS */
/* 46 C07, gpio_12, spi1_m_cs1, NA */
/* 47 B07, gpio_13, spi1_m_cs2, NA */
/* 48 A07, gpio_14, spi1_m_cs3, NA */
/* 49 B08, gpio_15, i2s_rxd, NA */
/* 50 A08, gpio_16, i2s_rscki, NA */
/* 51 B09, gpio_17, i2s_rws, NA */
/* 52 A09, gpio_18, i2s_tsck, NA */
/* 53 C09, gpio_19, i2s_twsi, NA */
/* 54 D09, gpio_20, i2s_txd, NA */
/* 55 D08, gpio_21, spi0_m_sck, NA */		/* SPI Flash */
/* 56 E07, gpio_22, spi0_m_miso, NA */		/* SPI Flash */
/* 57 E09, gpio_23, spi0_m_mosi, NA */		/* SPI Flash */
/* 58 E08, gpio_24, spi0_m_cs0, NA */		/* SPI Flash */
/* 59 A10, gpio_25, spi0_m_cs1, NA */
/* 60 B10, gpio_26, spi0_m_cs2, NA */
/* 61 C10, gpio_27, spi0_m_cs3, NA */
/* 62 D10, gpio_28, NA, NA */
/* 63 E10, gpio_ss_10, pwm_0, NA */
/* 64 D11, gpio_ss_11, pwm_1, NA */
/* 65 C11, gpio_ss_12, pwm_2, NA */
/* 66 B11, gpio_ss_13, pwm_3, NA */
/* 67 D12, gpio_ss_14, clkout_32khz, NA */
/* 68 C12, gpio_ss_15, clkout_16mhz, NA */

/*
 * On the QUARK_SE platform there are a minimum of 69 pins that can be possibly
 * set.  This would be a total of 5 registers to store the configuration as per
 * the bit description from above
 */
#define PINMUX_MAX_REGISTERS	5
static void _pinmux_defaults(uint32_t base)
{
	uint32_t mux_config[PINMUX_MAX_REGISTERS] = { 0, 0, 0, 0, 0};
	int i = 0;

	PIN_CONFIG(mux_config,  0, PINMUX_FUNC_B);
	PIN_CONFIG(mux_config,  1, PINMUX_FUNC_B);
	PIN_CONFIG(mux_config,  2, PINMUX_FUNC_B);
	PIN_CONFIG(mux_config,  3, PINMUX_FUNC_B);
	PIN_CONFIG(mux_config,  8, PINMUX_FUNC_C);
	PIN_CONFIG(mux_config,  9, PINMUX_FUNC_C);
	PIN_CONFIG(mux_config, 16, PINMUX_FUNC_C);
	PIN_CONFIG(mux_config, 17, PINMUX_FUNC_C);
	PIN_CONFIG(mux_config, 33, PINMUX_FUNC_B);
	PIN_CONFIG(mux_config, 40, PINMUX_FUNC_B);
	PIN_CONFIG(mux_config, 41, PINMUX_FUNC_B);
	PIN_CONFIG(mux_config, 42, PINMUX_FUNC_B);
	PIN_CONFIG(mux_config, 43, PINMUX_FUNC_B);
	PIN_CONFIG(mux_config, 44, PINMUX_FUNC_B);
	PIN_CONFIG(mux_config, 55, PINMUX_FUNC_B);
	PIN_CONFIG(mux_config, 56, PINMUX_FUNC_B);
	PIN_CONFIG(mux_config, 57, PINMUX_FUNC_B);
	PIN_CONFIG(mux_config, 58, PINMUX_FUNC_B);

	for (i = 0; i < PINMUX_MAX_REGISTERS; i++) {
		PRINT("PINMUX: configuring register i=%d reg=%x", i,
		      mux_config[i]);
		sys_write32(mux_config[i], PINMUX_SELECT_REGISTER(base, i));
	}
}

static inline void _pinmux_pullups(uint32_t base_address) { };

static uint32_t _quark_se_set_mux(uint32_t base, uint32_t pin, uint8_t func)
{
	/*
	 * the registers are 32-bit wide, but each pin requires 1 bit
	 * to set the input enable bit.
	 */
	uint32_t register_offset = (pin / 32) * 4;
	/*
	 * Now figure out what is the full address for the register
	 * we are looking for.  Add the base register to the register_mask
	 * Valid values include: 0x900, 0x904, 0x908, and 0x90C
	 */
	volatile uint32_t *mux_register = (uint32_t *)(base + register_offset);

	/*
	 * Finally grab the pin offset within the register
	 */
	uint32_t pin_offset = pin % 32;

	/*
	 * MAGIC NUMBER: 0x1 is used as the pullup is a single bit in a
	 * 32-bit register.
	 */
	(*(mux_register)) = ((*(mux_register)) & ~(0x1 << pin_offset)) |
		(func << pin_offset);

	return DEV_OK;
}

#ifdef CONFIG_PINMUX_DEV
static uint32_t pinmux_dev_set(struct device *dev, uint32_t pin, uint8_t func)
{
	const struct pinmux_config *pmux = dev->config->config_info;

	/*
	 * the registers are 32-bit wide, but each pin requires 2 bits
	 * to set the mode (A, B, C, or D).  As such we only get 16
	 * pins per register... hence the math for the register mask.
	 */
	uint32_t register_offset = (pin >> 4);
	/*
	 * Now figure out what is the full address for the register
	 * we are looking for.  Add the base register to the register_mask
	 */
	volatile uint32_t *mux_register =
		(uint32_t *)PINMUX_SELECT_REGISTER(pmux->base_address,
						   register_offset);

	/*
	 * Finally grab the pin offset within the register
	 */
	uint32_t pin_no = pin % 16;

	/*
	 * The value 3 is used because that is 2-bits for the mode of each
	 * pin.  The value 2 repesents the bits needed for each pin's mode.
	 */
	uint32_t pin_mask = MASK_2_BITS << (pin_no << 1);
	uint32_t mode_mask = mode << (pin_no << 1);
	(*(mux_register)) = ((*(mux_register)) & ~pin_mask) | mode_mask;

	return DEV_OK;
}

static uint32_t pinmux_dev_get(struct device *dev, uint32_t pin, uint8_t *func)
{
	const struct pinmux_config *pmux = dev->config->config_info;

	/*
	 * the registers are 32-bit wide, but each pin requires 2 bits
	 * to set the mode (A, B, C, or D).  As such we only get 16
	 * pins per register... hence the math for the register mask.
	 */
	uint32_t register_offset = pin >> 4;
	/*
	 * Now figure out what is the full address for the register
	 * we are looking for.  Add the base register to the register_mask
	 */
	volatile uint32_t *mux_register =
		(uint32_t *)PINMUX_SELECT_REGISTER(pmux->base_address,
						   register_offset);

	/*
	 * Finally grab the pin offset within the register
	 */
	uint32_t pin_no = pin % 16;

	/*
	 * The value 3 is used because that is 2-bits for the mode of each
	 * pin.  The value 2 repesents the bits needed for each pin's mode.
	 */
	uint32_t pin_mask = MASK_2_BITS << (pin_no << 1);
	uint32_t mode_mask = (*(mux_register)) & pin_mask;
	uint32_t mode = mode_mask >> (pin_no << 1);

	*func = mode;

	return DEV_OK;
}
#else
static uint32_t pinmux_dev_set(struct device *dev, uint32_t pin, uint8_t func)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(pin);
	ARG_UNUSED(func);

	PRINT("ERROR: %s is not enabled", __func__);

	return DEV_NOT_CONFIG;
}

static uint32_t pinmux_dev_get(struct device *dev, uint32_t pin, uint8_t *func)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(pin);
	ARG_UNUSED(func);

	PRINT("ERROR: %s is not enabled", __func__);

	return DEV_NOT_CONFIG;
}
#endif /* CONFIG_PINMUX_DEV */

static uint32_t pinmux_dev_pullup(struct device *dev,
				  uint32_t pin,
				  uint8_t func)
{
	const struct pinmux_config *pmux = dev->config->config_info;

	_quark_se_set_mux(pmux->base_address + PINMUX_PULLUP_OFFSET,
			  pin, func);

	return DEV_OK;
}
static uint32_t pinmux_dev_input(struct device *dev,
				 uint32_t pin, uint8_t func)
{
	const struct pinmux_config *pmux = dev->config->config_info;

	_quark_se_set_mux(pmux->base_address + PINMUX_INPUT_OFFSET, pin, func);

	return DEV_OK;
}

static struct pinmux_driver_api api_funcs = {
	.set = pinmux_dev_set,
	.get = pinmux_dev_get,
	.pullup = pinmux_dev_pullup,
	.input = pinmux_dev_input
};

int pinmux_initialize(struct device *port)
{
	const struct pinmux_config *pmux = port->config->config_info;

	port->driver_api = &api_funcs;

	_pinmux_defaults(pmux->base_address);
	_pinmux_pullups(pmux->base_address);

	return DEV_OK;
}

struct pinmux_config board_pmux = {
	.base_address = CONFIG_PINMUX_BASE,
};

DEVICE_INIT(pmux, PINMUX_NAME, &pinmux_initialize,
			NULL, &board_pmux,
			SECONDARY, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
