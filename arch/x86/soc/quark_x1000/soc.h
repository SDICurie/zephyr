/*
 * Copyright (c) 2013-2015, Wind River Systems, Inc.
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

/**
 * @file
 * @brief Board configuration macros for the Quark X1000 SoC
 *
 * This header file is used to specify and describe SoC-level aspects for
 * the Quark X1000 SoC.
 */

#ifndef __SOC_H_
#define __SOC_H_

#include <misc/util.h>

#ifndef _ASMLANGUAGE
#include <device.h>
#include <drivers/rand32.h>
#endif

#ifdef CONFIG_IOAPIC
#include <drivers/ioapic.h>
#if defined(CONFIG_UART_IRQ_FALLING_EDGE)
	#define UART_IRQ_FLAGS (IOAPIC_EDGE | IOAPIC_LOW)
#elif defined(CONFIG_UART_IRQ_RISING_EDGE)
	#define UART_IRQ_FLAGS (IOAPIC_EDGE | IOAPIC_HIGH)
#elif defined(CONFIG_UART_IRQ_LEVEL_HIGH)
	#define UART_IRQ_FLAGS (IOAPIC_LEVEL | IOAPIC_HIGH)
#elif defined(CONFIG_UART_IRQ_LEVEL_LOW)
	#define UART_IRQ_FLAGS (IOAPIC_LEVEL | IOAPIC_LOW)
#endif
#endif /* CONFIG_IOAPIC */

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_STD_IRQS 16   /* number of "standard" IRQs on an x86 platform */
#define INT_VEC_IRQ0 0x20 /* Vector number for IRQ0 */

/*
 * The IRQ_CONNECT() API connects to a (virtualized) IRQ and the
 * associated interrupt controller is programmed with the allocated vector.
 * The Quark board virtualizes IRQs as follows:
 *
 *   - The first CONFIG_IOAPIC_NUM_RTES IRQs are provided by the IOAPIC
 *   - The remaining IRQs are provided by the LOAPIC.
 *
 * Thus, for example, if the IOAPIC supports 24 IRQs:
 *
 *   - IRQ0 to IRQ23   map to IOAPIC IRQ0 to IRQ23
 *   - IRQ24 to IRQ29  map to LOAPIC LVT entries as follows:
 *
 *       IRQ24 -> LOAPIC_TIMER
 *       IRQ25 -> LOAPIC_THERMAL
 *       IRQ26 -> LOAPIC_PMC
 *       IRQ27 -> LOAPIC_LINT0
 *       IRQ28 -> LOAPIC_LINT1
 *       IRQ29 -> LOAPIC_ERROR
 */

/* PCI definitions */
#define PCI_BUS_NUMBERS 2

#define PCI_CTRL_ADDR_REG 0xCF8
#define PCI_CTRL_DATA_REG 0xCFC

#define PCI_INTA 1
#define PCI_INTB 2
#define PCI_INTC 3
#define PCI_INTD 4


/**
 *
 * @brief Convert PCI interrupt PIN to IRQ
 *
 * The routine uses "standard design consideration" and implies that
 * INTA (pin 1) -> IRQ 16
 * INTB (pin 2) -> IRQ 17
 * INTC (pin 3) -> IRQ 18
 * INTD (pin 4) -> IRQ 19
 *
 * In case a mini-PCIe card is used, the IRQs are swizzled:
 * INTA (pin 1) -> IRQ 17
 * INTB (pin 2) -> IRQ 18
 * INTC (pin 3) -> IRQ 19
 * INTD (pin 4) -> IRQ 16
 *
 * @return IRQ number, -1 if the result is incorrect
 *
 */

static inline int pci_pin2irq(int bus, int dev, int pin)
{
	if (bus < 0 || bus > 1)
		return -1;
	if ((pin < PCI_INTA) || (pin > PCI_INTD))
		return -1;
	return NUM_STD_IRQS + ((pin - 1 + bus) & 3);
}

#ifdef __cplusplus
}
#endif

#endif /* __SOC_H_ */
