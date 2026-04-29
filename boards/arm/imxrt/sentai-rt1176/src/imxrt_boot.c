/****************************************************************************
 * boards/arm/imxrt/sentai-rt1176/src/imxrt_boot.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "imxrt_start.h"
#include "sentai-rt1176.h"
#include "arm_internal.h"
#include "nvic.h"
#ifdef CONFIG_BOOT_RUNFROMFLASH
#  include "imxrt_flexspi_nor_boot.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_ocram_initialize
 *
 * Description:
 *   Called off reset vector to reconfigure the flexRAM
 *   and finish the FLASH to RAM Copy.
 *
 *   Only meaningful for FlexSPI-XIP boot. For RAM-loaded images
 *   (CONFIG_BOOT_RUNFROMISRAM) the loader has already placed all
 *   sections, so the function is a no-op.
 *
 ****************************************************************************/

void imxrt_ocram_initialize(void)
{
#ifdef CONFIG_BOOT_RUNFROMFLASH
  const uint32_t *src;
  uint32_t *dest;
  uint32_t regval;

  /* Reallocate 128K of Flex RAM from ITCM to OCRAM
   * Final Configuration is
   *    128 DTCM
   *
   *    128 FlexRAM OCRAM  (202C:0000-202D:ffff)
   *    256 FlexRAM OCRAM  (2028:0000-202B:ffff)
   *    512 System  OCRAM2 (2020:0000-2027:ffff)
   * */

  putreg32(0xaa555555, IMXRT_IOMUXC_GPR_GPR17);
  regval = getreg32(IMXRT_IOMUXC_GPR_GPR16);
  putreg32(regval | GPR_GPR16_FLEXRAM_BANK_CFG_SEL, IMXRT_IOMUXC_GPR_GPR16);

  src = (uint32_t *) (LOCATE_IN_SRC(g_boot_data.start) + g_boot_data.size);
  dest = (uint32_t *) (g_boot_data.start + g_boot_data.size);

  while (dest < (uint32_t *) &_etext)
    {
      *dest++ = *src++;
    }
#endif
}

/****************************************************************************
 * Name: imxrt_flexram_partition
 *
 * Description:
 *   Sets FlexRAM partitioning
 *
 ****************************************************************************/

void imxrt_flexram_partition(void)
{
}

/****************************************************************************
 * Name: board_reset
 *
 * Description:
 *   board_reset() is exported to NuttX (BOARDCTL_RESET) and triggers a
 *   system reset via the Cortex-M SCB AIRCR SYSRESETREQ bit. Returns 0
 *   on success; in practice it does not return because the SoC resets.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_RESET
int board_reset(int status)
{
  UNUSED(status);

  __asm__ __volatile__ ("dsb sy" ::: "memory");
  putreg32(NVIC_AIRCR_VECTKEY | NVIC_AIRCR_SYSRESETREQ, NVIC_AIRCR);
  __asm__ __volatile__ ("dsb sy" ::: "memory");

  for (; ; )
    {
      __asm__ __volatile__ ("wfi");
    }

  return 0;
}
#endif

/****************************************************************************
 * Name: imxrt_boardinitialize
 *
 * Description:
 *   All i.MX RT architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after clocking and
 *   memory have been configured but before caches have been enabled and
 *   before any devices have been initialized.
 *
 ****************************************************************************/

void imxrt_boardinitialize(void)
{
  /* Defensive: the coralmicro flashtool elfloader leaves SysTick
   * running with its own (now-stale) configuration. NuttX's
   * up_irqinitialize() unmasks IRQs at the end (`cpsie i`) before
   * up_timer_initialize() installs our SysTick handler, so any
   * leftover SysTick exception (live or pending) fires straight into
   * irq_unexpected_isr -> assert.
   *
   * Mitigations, in order:
   *   1. Disable SysTick (CTRL/RELOAD/CURRENT all zero).
   *   2. Clear the SysTick pending bit in ICSR (PENDSTCLR = bit 25).
   *      A pending exception survives the CTRL clear; ICSR is the
   *      only way to drop it without a real timer fire.
   *   3. DSB/ISB so the write retires before any subsequent code
   *      can re-enable interrupts.
   */

  putreg32(0, NVIC_SYSTICK_CTRL);
  putreg32(0, NVIC_SYSTICK_RELOAD);
  putreg32(0, NVIC_SYSTICK_CURRENT);
  putreg32(NVIC_INTCTRL_PENDSTCLR, NVIC_INTCTRL);
  __asm__ __volatile__ ("dsb sy" ::: "memory");
  __asm__ __volatile__ ("isb sy" ::: "memory");

  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  imxrt_autoled_initialize();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called immediately after up_intitialize() is called and just before
 *   the initial application is started.  This additional initialization
 *   phase may be used, for example, to initialize board-specific device
 *   drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  /* Perform board initialization */

  imxrt_bringup();
}
#endif /* CONFIG_BOARD_LATE_INITIALIZE */
