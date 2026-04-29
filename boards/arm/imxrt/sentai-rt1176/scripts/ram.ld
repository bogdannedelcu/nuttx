/****************************************************************************
 * boards/arm/imxrt/sentai-rt1176/scripts/ram.ld
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

/* Pure-RAM linker for the Sentai RT1176.
 *
 * Used when the image is loaded straight into RAM by an external loader
 * (currently coralmicro flashtool.py --ram, which targets NXP's SDP +
 * elfloader path). No FlexSPI XIP, no IVT/DCD, no copy-from-NOR step at
 * startup. Mirrors the layout sentai_runtime uses — see
 * coralmicro/examples/sentai_runtime/MIMXRT1176xxxxx_cm7_ram_mp.ld.
 */

MEMORY
{
  /* Skip the first 0x800 bytes of ITCM — the RT1176 ROM bootloader and
   * SDP/elfloader land short stubs there during the load handshake.
   */

  itcm  (rwx) : ORIGIN = 0x00000800, LENGTH = 256K - 0x800
  dtcm  (rwx) : ORIGIN = 0x20000000, LENGTH = 256K
  ocram (rwx) : ORIGIN = 0x20240000, LENGTH = 1024K  /* OCRAM1 + OCRAM2 */
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)

SECTIONS
{
    .text : ALIGN(4)
    {
        _stext = ABSOLUTE(.);
        *(.vectors)
        *(.text .text.*)
        *(.fixup)
        *(.gnu.warning)
        *(.rodata .rodata.*)
        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        *(.got)
        *(.gcc_except_table)
        *(.gnu.linkonce.r.*)
        _etext = ABSOLUTE(.);
    } > itcm

    .init_section :
    {
        _sinit = ABSOLUTE(.);
        *(.init_array .init_array.*)
        _einit = ABSOLUTE(.);
    } > itcm

    .ARM.extab :
    {
        *(.ARM.extab*)
    } > itcm

    .ARM.exidx :
    {
        __exidx_start = ABSOLUTE(.);
        *(.ARM.exidx*)
        __exidx_end = ABSOLUTE(.);
    } > itcm

    _eronly = ABSOLUTE(.);

    .data :
    {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > dtcm

    .ramfunc ALIGN(4):
    {
        _sramfuncs = ABSOLUTE(.);
        *(.ramfunc  .ramfunc.*)
        _eramfuncs = ABSOLUTE(.);
    } > itcm

    /* For pure-RAM build .data already lives in dtcm at link time, so
     * the C startup "copy from _eronly to _sdata" is a self-copy. The
     * arch start code skips it when LOADADDR == VMA. We still publish
     * _framfuncs / _ramfuncs symbols for source compatibility.
     */

    _framfuncs = ADDR(.ramfunc);

    .bss :
    {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > dtcm

    /* Stabs debugging sections. */

    .stab 0 : { *(.stab) }
    .stabstr 0 : { *(.stabstr) }
    .stab.excl 0 : { *(.stab.excl) }
    .stab.exclstr 0 : { *(.stab.exclstr) }
    .stab.index 0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment 0 : { *(.comment) }
    .debug_abbrev 0 : { *(.debug_abbrev) }
    .debug_info 0 : { *(.debug_info) }
    .debug_line 0 : { *(.debug_line) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    .debug_aranges 0 : { *(.debug_aranges) }

    /* Heap will start where bss ends in dtcm and may extend into ocram
     * via the architecture's allocateheap path. _ram_size is the upper
     * bound the kernel uses for its primary heap region.
     */

    _boot_loadaddr = 0;
    _boot_size = 0;
    _ram_size = LENGTH(dtcm);
}
