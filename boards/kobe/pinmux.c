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
#include "quark_se_pinmux_common.h"

#define PINMUX_SELECT_OFFSET    0x30

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
/* 4  K06, gpio_4, ain_4, NA */
/* 5  L06, gpio_5, ain_5, NA */
/* 6  H04, gpio_6, ain_6, NA */
/* 7  G03, gpio_7, ain_7, NA */
/* 8  L05, gpio_ss_0, ain_8, uart1_cts */
/* 9  M05, gpio_ss_1, ain_9, uart1_rts */
/* 10 K05, gpio_ss_2, ain_10 */				/* AD0 */
/* 11 G01, gpio_ss_3, ain_11 */				/* AD1 */
/* 12 J04, gpio_ss_4, ain_12 */				/* AD2 */
/* 13 G02, gpio_ss_5, ain_13 */				/* AD3 */
/* 14 F01, gpio_ss_6, ain_14 */				/* AD4 */
/* 15 J05, gpio_ss_7, ain_15 */				/* AD5 */
/* 16 L04, gpio_ss_8, ain_16, uart1_txd */		/* IO1 */
/* 17 M04, gpio_ss_9, ain_17, uart1_rxd */		/* IO0 */
/* 18 K04, uart0_rx, ain_18, NA */
/* 19 B02, uart0_tx, gpio_31, NA */
/* 20 C01, i2c0_scl, NA, NA */
/* 21 C02, i2c0_sda, NA, NA */
/* 22 D01, i2c1_scl, NA, NA */
/* 23 D02, i2c1_sda, NA, NA */
/* 24 E01, i2c0_ss_sda, NA, NA */
/* 25 E02, i2c0_ss_scl, NA, NA */
/* 26 B03, i2c1_ss_sda, NA, NA */
/* 27 A03, i2c1_ss_scl, NA, NA */
/* 28 C03, spi0_ss_miso, NA, NA */
/* 29 E03, spi0_ss_mosi, NA, NA */
/* 30 D03, spi0_ss_sck, NA, NA */
/* 31 D04, spi0_ss_cs0, NA, NA */
/* 32 C04, spi0_ss_cs1, NA, NA */
/* 33 B04, spi0_ss_cs2, gpio_29, NA */
/* 34 A04, spi0_ss_cs3, gpio_30, NA */
/* 35 B05, spi1_ss_miso, NA, NA */
/* 36 C05, spi1_ss_mosi, NA, NA */
/* 37 D05, spi1_ss_sck, NA, NA */
/* 38 E05, spi1_ss_cs0, NA, NA */
/* 39 E04, spi1_ss_cs1, NA, NA */
/* 40 A06, spi1_ss_cs2, uart0_cts, NA */
/* 41 B06, spi1_ss_cs3, uart0_rts, NA */
/* 42 C06, gpio_8, spi1_m_sck, NA */			/* IO13 */
/* 43 D06, gpio_9, spi1_m_miso, NA */			/* IO12 */
/* 44 E06, gpio_10, spi1_m_mosi, NA */			/* IO11 */
/* 45 D07, gpio_11, spi1_m_cs0, NA */			/* IO10 */
/* 46 C07, gpio_12, spi1_m_cs1, NA */
/* 47 B07, gpio_13, spi1_m_cs2, NA */
/* 48 A07, gpio_14, spi1_m_cs3, NA */
/* 49 B08, gpio_15, i2s_rxd, NA */			/* IO5 */
/* 50 A08, gpio_16, i2s_rscki, NA */			/* IO8 */
/* 51 B09, gpio_17, i2s_rws, NA */			/* IO3 */
/* 52 A09, gpio_18, i2s_tsck, NA */			/* IO2 */
/* 53 C09, gpio_19, i2s_twsi, NA */			/* IO4 */
/* 54 D09, gpio_20, i2s_txd, NA */			/* IO7 */
/* 55 D08, gpio_21, spi0_m_sck, NA */
/* 56 E07, gpio_22, spi0_m_miso, NA */
/* 57 E09, gpio_23, spi0_m_mosi, NA */
/* 58 E08, gpio_24, spi0_m_cs0, NA */
/* 59 A10, gpio_25, spi0_m_cs1, NA */
/* 60 B10, gpio_26, spi0_m_cs2, NA */
/* 61 C10, gpio_27, spi0_m_cs3, NA */
/* 62 D10, gpio_28, NA, NA */
/* 63 E10, gpio_ss_10, pwm_0, NA */			/* IO3 */
/* 64 D11, gpio_ss_11, pwm_1, NA */			/* IO5 */
/* 65 C11, gpio_ss_12, pwm_2, NA */			/* IO6 */
/* 66 B11, gpio_ss_13, pwm_3, NA */			/* IO9 */
/* 67 D12, gpio_ss_14, clkout_32khz, NA */
/* 68 C12, gpio_ss_15, clkout_16mhz, NA */
static void pinmux_sharkjumper_board(uint32_t *mux_config)
{
	/*
	 * While it is not necessary to specify the pinmux for function A,
	 * since that is the default, they are put here explicitly for
	 * documentation purposes.
	 */

	/* GPIO[0]/AIN[0]/SPI_S_CS_B */
	PIN_CONFIG(mux_config, 0, PINMUX_FUNC_B);
	/* GPIO[1]/AIN[1]/SPI_S_MISO */
	PIN_CONFIG(mux_config, 1, PINMUX_FUNC_B);
	/* GPIO[2]/AIN[2]/SPI_S_SCK */
	PIN_CONFIG(mux_config, 2, PINMUX_FUNC_B);
	/* GPIO[3]/AIN[3]/SPI_S_MOSI */
	PIN_CONFIG(mux_config, 3, PINMUX_FUNC_B);
	/* GPIO[4]/AIN[4] */
	PIN_CONFIG(mux_config, 4, PINMUX_FUNC_B);
	/* GPIO[5]/AIN[5] */
	PIN_CONFIG(mux_config, 5, PINMUX_FUNC_B);
	/* GPIO[6]/AIN[6] - BLE_SWDIO */
	PIN_CONFIG(mux_config, 6, PINMUX_FUNC_A);
	/* GPIO[7]/AIN[7] */
	PIN_CONFIG(mux_config, 7, PINMUX_FUNC_B);
	/* GPIO_SS[0]/AIN[8]/UART1_CTS_B */
	PIN_CONFIG(mux_config, 8, PINMUX_FUNC_C);
	/* GPIO_SS[1]/AIN[9]/UART1_RTS_B */
	PIN_CONFIG(mux_config, 9, PINMUX_FUNC_C);
	/* GPIO_SS[2]/AIN[10] - BATT_FG_EN */
	PIN_CONFIG(mux_config, 10, PINMUX_FUNC_A);
	/* GPIO_SS[3]/AIN[11] - GPS_WAKEUP */
	PIN_CONFIG(mux_config, 11, PINMUX_FUNC_A);
	/* GPIO_SS[4]/AIN[12] - GPS_ON_OFF */
	PIN_CONFIG(mux_config, 12, PINMUX_FUNC_A);

	PIN_CONFIG(mux_config, 13, PINMUX_FUNC_A);
	/* GPIO_SS[6]/AIN[14] - GPS_RESET */
	PIN_CONFIG(mux_config, 14, PINMUX_FUNC_A);
	/* GPIO_SS[7]/AIN[15] */
	PIN_CONFIG(mux_config, 15, PINMUX_FUNC_A);
	/* GPIO_SS[8]/AIN[16]/UART1_TXD */
	PIN_CONFIG(mux_config, 16, PINMUX_FUNC_C);
	/* GPIO_SS[9]/AIN[17]/UART1_RXD */
	PIN_CONFIG(mux_config, 17, PINMUX_FUNC_C);
	/* UART0_RXD/AIN[18] */
	PIN_CONFIG(mux_config, 18, PINMUX_FUNC_A);
	/* UART0_TXD/AIN[19] */
	PIN_CONFIG(mux_config, 19, PINMUX_FUNC_A);
	/* I2C0_SCL - LED Driver */
	PIN_CONFIG(mux_config, 20, PINMUX_FUNC_A);
	/* I2C0_SDA - LED Driver */
	PIN_CONFIG(mux_config, 21, PINMUX_FUNC_A);
	/* I2C0_SS_SDA - J2-PIN5 */
	PIN_CONFIG(mux_config, 24, PINMUX_FUNC_A);
	/* I2C0_SS_SCL - J2-PIN4 */
	PIN_CONFIG(mux_config, 25, PINMUX_FUNC_A);
	/* I2C1_SS_SDA - BARO BME280 */
	PIN_CONFIG(mux_config, 26, PINMUX_FUNC_A);
	/* I2C1_SS_SCL - BARO BME280 */
	PIN_CONFIG(mux_config, 27, PINMUX_FUNC_A);
	/* SPI0_SS_MISO */
	PIN_CONFIG(mux_config, 28, PINMUX_FUNC_A);
	/* SPI0_SS_MOSI */
	PIN_CONFIG(mux_config, 29, PINMUX_FUNC_A);
	/* SPI0_SS_SCK */
	PIN_CONFIG(mux_config, 30, PINMUX_FUNC_A);
	/* SPI0_SS_CS_B[0] - 24G ACCEL CS on SJ3.0 */
	PIN_CONFIG(mux_config, 31, PINMUX_FUNC_A);
	/* SPI0_SS_CS_B[1] - GPS CS */
	PIN_CONFIG(mux_config, 32, PINMUX_FUNC_A);
	/* SPI0_SS_CS_B[2]/GPIO[29] - External SPI Connector CS on SJ3.0 */
	PIN_CONFIG(mux_config, 33, PINMUX_FUNC_A);
	/* SPI0_SS_CS_B[3]/GPIO[30] - Charging Enable */
	PIN_CONFIG(mux_config, 34, PINMUX_FUNC_B);
	/* SPI1_SS_MISO */
	PIN_CONFIG(mux_config, 35, PINMUX_FUNC_A);
	/* SPI1_SS_MOSI */
	PIN_CONFIG(mux_config, 36, PINMUX_FUNC_A);
	/* SPI1_SS_SCK */
	PIN_CONFIG(mux_config, 37, PINMUX_FUNC_A);
	/* SPI1_SS_CS_B[0] */
	PIN_CONFIG(mux_config, 38, PINMUX_FUNC_A);
	/* SPI1_SS_CS_B[1] */
	PIN_CONFIG(mux_config, 39, PINMUX_FUNC_A);
	/* SPI1_SS_CS_B[2]/UART0_CTS_B */
	PIN_CONFIG(mux_config, 40, PINMUX_FUNC_B);
	/* SPI1_SS_CS_B[3]/UART0_RTS_B */
	PIN_CONFIG(mux_config, 41, PINMUX_FUNC_B);
	/* GPIO[8]/SPI1_M_SCK */
	PIN_CONFIG(mux_config, 42, PINMUX_FUNC_B);
	/* GPIO[9]/SPI1_M_MISO */
	PIN_CONFIG(mux_config, 43, PINMUX_FUNC_B);
	/* GPIO[10]/SPI1_M_MOSI */
	PIN_CONFIG(mux_config, 44, PINMUX_FUNC_B);
	/* GPIO[11]/SPI1_M_CS_B[0] - UHF_CS0 on SJ3.0 */
	PIN_CONFIG(mux_config, 45, PINMUX_FUNC_A);
	/* GPIO[12]/SPI1_M_CS_B[1] - UHF_GPIO3 on SJ3.0 */
	PIN_CONFIG(mux_config, 46, PINMUX_FUNC_B);
	/* GPIO[13]/SPI1_M_CS_B[2] - UHF_GPIO2 on SJ3.0 */
	PIN_CONFIG(mux_config, 47, PINMUX_FUNC_B);
	/* GPIO[14]/SPI1_M_CS_B[3] - SJ2XB_IRQ on SJSupreme - Optional */
	PIN_CONFIG(mux_config, 48, PINMUX_FUNC_B);
	/* GPIO[21]/SPI0_M_SCK */
	PIN_CONFIG(mux_config, 55, PINMUX_FUNC_B);
	/* GPIO[22]/SPI0_M_MISO */
	PIN_CONFIG(mux_config, 56, PINMUX_FUNC_B);
	/* GPIO[23]/SPI0_M_MOSI */
	PIN_CONFIG(mux_config, 57, PINMUX_FUNC_B);
	/* GPIO[24]/SPI0_M_CS_0 - SPI FLASH CS on SJ3.0 or MAX3107 CS on SJ Supreme */
	PIN_CONFIG(mux_config, 58, PINMUX_FUNC_A);
	/* GPIO[25]/SPI0_M_CS_1 - 1GB_W# on SJ3.0 */
	PIN_CONFIG(mux_config, 59, PINMUX_FUNC_A);
	/* GPIO[26]/SPI0_M_CS_2 - 1GB_HOLD# on SJ3.0 */
	PIN_CONFIG(mux_config, 60, PINMUX_FUNC_A);
	/* GPIO[27]/SPI0_M_CS[3] - BLE_SW_CLK */
	PIN_CONFIG(mux_config, 61, PINMUX_FUNC_A);
	/* GPIO_SS[10]/PWM[0] - MAG_INT */
	PIN_CONFIG(mux_config, 63, PINMUX_FUNC_A);
	/* GPIO_SS[11]/PWM[1] */
	PIN_CONFIG(mux_config, 64, PINMUX_FUNC_B);
	/* GPIO_SS[12]/PWM[2] */
	PIN_CONFIG(mux_config, 65, PINMUX_FUNC_B);
	/* GPIO_SS[13]/PWM[3] */
	PIN_CONFIG(mux_config, 66, PINMUX_FUNC_B);
	/* sss gpio1 bit 6 */
	PIN_CONFIG(mux_config, 67, PINMUX_FUNC_A);
	/* sss gpio1 bit 7 */
	PIN_CONFIG(mux_config, 68, PINMUX_FUNC_A);
}

static uint32_t mux_config[PINMUX_MAX_REGISTERS] = { 0, 0, 0, 0, 0 };

struct pinmux_config board_pmux = {
	.base_address = CONFIG_PINMUX_BASE,
};

int pinmux_initialize(struct device *port)
{
	int i = 0;

	quark_se_pinmux_initialize_common(port, mux_config);
	pinmux_sharkjumper_board(mux_config);

	for (i = 0; i < PINMUX_MAX_REGISTERS; i++) {
		sys_write32(mux_config[i],
			    PINMUX_SELECT_REGISTER(board_pmux.base_address, i));
	}
	return DEV_OK;
}

DEVICE_INIT(pmux,                   /* config name */
	    PINMUX_NAME,                        /* driver name */
	    &pinmux_initialize,                 /* init function */
	    NULL,
	    &board_pmux,                        /* config options*/
	    SECONDARY,
	    CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
