/****************************************************************************
 * boards/arm/imxrt/sentai-rt1176/src/imxrt_flexspi_nand.c
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
 * MTD driver for the on-board Winbond W25N01-class FlexSPI NAND of the
 * Sentai RT1176 custom board.
 *
 * Geometry (validated against the coralmicro reference firmware -- see
 *  ~/work/coralmicro/libs/base/fx_nand_driver.h):
 *
 *    1024 blocks total                      (~128 MiB raw)
 *    64 pages / block
 *    2048 B data per page  (we ignore on-chip OOB; ECC is internal)
 *
 *    User partition we expose to the FS layer: blocks
 *    SENTAI_NAND_USER_BLK_FIRST..SENTAI_NAND_USER_BLK_LAST inclusive.
 *
 * Wire protocol:  single-SPI W25N command set over FlexSPI1 port A1.
 *
 *    0xFF  Reset
 *    0x9F  Read JEDEC ID
 *    0x0F  Get feature       (read status)
 *    0x1F  Set feature       (used to unlock all blocks at boot)
 *    0x06  Write enable
 *    0x13  Page read to cache  (row addr)
 *    0x03  Read from cache     (col addr, 8 dummy cycles)
 *    0x02  Program load        (col addr)
 *    0x10  Program execute     (row addr)
 *    0xD8  Block erase         (row addr -- any page in block)
 *
 * The driver supports MTD geometry for the user partition and converts
 * sector-level MTD calls into NAND page transactions.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <nuttx/debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/signal.h>

#include "arm_internal.h"     /* up_mdelay / up_udelay */
#include "imxrt_flexspi.h"
#include "imxrt_iomuxc.h"
#include "imxrt_gpio.h"
#include "hardware/imxrt_pinmux.h"
#include "sentai-rt1176.h"

#ifdef CONFIG_SENTAI_RT1176_FLEXSPI_NAND

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define NAND_PAGE_DATA_SIZE      2048
#define NAND_PAGES_PER_BLOCK     64
#define NAND_TOTAL_BLOCKS        1024
#define NAND_BLOCK_SIZE          (NAND_PAGE_DATA_SIZE * NAND_PAGES_PER_BLOCK)

/* User partition (matching coralmicro fx_nand_driver.h). */

#define NAND_USER_BLK_FIRST      76
#define NAND_USER_BLK_LAST       523
#define NAND_USER_BLK_COUNT      (NAND_USER_BLK_LAST - NAND_USER_BLK_FIRST + 1)

/* Status / feature register addresses (W25N "Get/Set Feature"). */

#define NAND_REG_PROTECTION      0xa0
#define NAND_REG_CONFIG          0xb0
#define NAND_REG_STATUS          0xc0

/* Status bits (NAND_REG_STATUS). */

#define NAND_SR_BUSY             (1u << 0)
#define NAND_SR_WEL              (1u << 1)
#define NAND_SR_EFAIL            (1u << 2)  /* erase failure */
#define NAND_SR_PFAIL            (1u << 3)  /* program failure */
#define NAND_SR_ECC_MASK         (3u << 4)
#define NAND_SR_ECC_UNCORR       (2u << 4)

/* LUT slot indices. */

enum
{
  LUT_RESET = 0,
  LUT_READ_ID,
  LUT_GET_FEATURE,
  LUT_SET_FEATURE,
  LUT_WRITE_ENABLE,
  LUT_PAGE_READ,        /* 0x13 row addr */
  LUT_READ_CACHE,       /* 0x03 col addr + dummy + read */
  LUT_PROG_LOAD,        /* 0x02 col addr + write */
  LUT_PROG_EXEC,        /* 0x10 row addr */
  LUT_BLOCK_ERASE,      /* 0xD8 row addr */
  LUT_COUNT,
};

/****************************************************************************
 * Private types
 ****************************************************************************/

struct sentai_nand_dev_s
{
  struct mtd_dev_s            mtd;     /* MTD interface (must be first) */
  struct flexspi_dev_s       *flexspi; /* FlexSPI controller handle */
  enum flexspi_port_e         port;
  uint32_t                    base_addr;
};

/****************************************************************************
 * LUT table -- single-SPI, mirrors the coralmicro / NXP NAND_Flash_*
 * reference (data path is intentionally single-SPI; the chip's ECC is
 * what makes the read fast, not pad width).
 ****************************************************************************/

static const uint32_t g_nand_lut[][4] =
{
  [LUT_RESET] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR, FLEXSPI_1PAD, 0xff,
                    FLEXSPI_COMMAND_STOP, FLEXSPI_1PAD, 0x00),
  },

  [LUT_READ_ID] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,      FLEXSPI_1PAD, 0x9f,
                    FLEXSPI_COMMAND_DUMMY_SDR, FLEXSPI_1PAD, 0x08),
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_READ_SDR, FLEXSPI_1PAD, 0x03,
                    FLEXSPI_COMMAND_STOP,     FLEXSPI_1PAD, 0x00),
  },

  /* Get/Set Feature: opcode + register address are TWO consecutive
   * CMD_SDR ops, not opcode + RADDR. The "address" passed to the
   * sequencer is unused for these LUTs; the register byte is hard-
   * coded in the LUT itself, so we keep one LUT per (cmd, reg) pair.
   *
   * 0xA0 = NAND_REG_PROTECTION (block-protect bits)
   * 0xB0 = NAND_REG_CONFIG     (chip features)
   * 0xC0 = NAND_REG_STATUS     (busy/WEL/ECC/program-fail)
   *
   * We only need GET 0xC0 (status) and SET 0xA0 (unlock) for now.
   */

  [LUT_GET_FEATURE] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,      FLEXSPI_1PAD, 0x0f,
                    FLEXSPI_COMMAND_SDR,      FLEXSPI_1PAD, 0xc0),
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_READ_SDR, FLEXSPI_1PAD, 0x01,
                    FLEXSPI_COMMAND_STOP,     FLEXSPI_1PAD, 0x00),
  },

  [LUT_SET_FEATURE] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0x1f,
                    FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0xa0),
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_WRITE_SDR, FLEXSPI_1PAD, 0x01,
                    FLEXSPI_COMMAND_STOP,      FLEXSPI_1PAD, 0x00),
  },

  [LUT_WRITE_ENABLE] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,  FLEXSPI_1PAD, 0x06,
                    FLEXSPI_COMMAND_STOP, FLEXSPI_1PAD, 0x00),
  },

  [LUT_PAGE_READ] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0x13,
                    FLEXSPI_COMMAND_RADDR_SDR, FLEXSPI_1PAD, 0x18),
  },

  /* READ_FROM_CACHE: opcode + 16-bit column address, then 8 dummy
   * cycles, then bulk read. NB: column address goes to CADDR_SDR
   * (not RADDR which is for the row).
   */

  [LUT_READ_CACHE] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0x03,
                    FLEXSPI_COMMAND_CADDR_SDR, FLEXSPI_1PAD, 0x10),
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_DUMMY_SDR, FLEXSPI_1PAD, 0x08,
                    FLEXSPI_COMMAND_READ_SDR,  FLEXSPI_1PAD, 0x04),
  },

  /* PROG_LOAD: opcode + 16-bit column + bulk write. */

  [LUT_PROG_LOAD] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0x02,
                    FLEXSPI_COMMAND_CADDR_SDR, FLEXSPI_1PAD, 0x10),
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_WRITE_SDR, FLEXSPI_1PAD, 0x04,
                    FLEXSPI_COMMAND_STOP,      FLEXSPI_1PAD, 0x00),
  },

  [LUT_PROG_EXEC] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0x10,
                    FLEXSPI_COMMAND_RADDR_SDR, FLEXSPI_1PAD, 0x18),
  },

  [LUT_BLOCK_ERASE] =
  {
    FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0xd8,
                    FLEXSPI_COMMAND_RADDR_SDR, FLEXSPI_1PAD, 0x18),
  },
};

/* Device config -- minimal; the FlexSPI clock is taken from the board's
 * CCM root configured in imxrt_clockconfig.c.
 */

static const struct flexspi_device_config_s g_nand_config =
{
  .flexspi_root_clk        = 0,                /* runtime, see init */
  .flash_size              = (NAND_TOTAL_BLOCKS * NAND_BLOCK_SIZE) >> 10, /* KB */
  .cs_interval_unit        = FLEXSPI_CS_INTERVAL_UNIT1_SCK_CYCLE,
  .cs_interval             = 0,
  .cs_hold_time            = 3,
  .cs_setup_time           = 3,
  .data_valid_time         = 2,
  .columnspace             = 0,
  .enable_word_address     = false,
  .awr_seq_index           = 0,
  .awr_seq_number          = 0,
  .ard_seq_index           = LUT_READ_CACHE,
  .ard_seq_number          = 1,
  .ahb_write_wait_unit     = FLEXSPI_AHB_WRITE_WAIT_UNIT2_AHB_CYCLE,
  .ahb_write_wait_interval = 0,
  .enable_write_mask       = false,
};

static struct sentai_nand_dev_s g_nand_dev;

/****************************************************************************
 * Low-level FlexSPI command helpers
 ****************************************************************************/

static int sentai_nand_cmd(struct sentai_nand_dev_s *dev,
                           uint8_t seq_index,
                           uint32_t addr)
{
  struct flexspi_transfer_s xfer =
  {
    .device_address = addr,
    .port           = dev->port,
    .cmd_type       = FLEXSPI_COMMAND,
    .seq_index      = seq_index,
    .seq_number     = 1,
    .data           = NULL,
    .data_size      = 0,
  };

  return FLEXSPI_TRANSFER(dev->flexspi, &xfer);
}

/* Note: the register address (0xC0 for status, 0xA0 for protect) is
 * baked into the LUT itself rather than passed as device_address,
 * because the W25N "Get/Set Feature" command takes the register
 * address as a second 8-bit opcode byte rather than an address phase.
 * This matches the NXP MfgTool / SDK reference (flexspi_nand_config_
 * MIMXRT1176.c). For now we only read NAND_REG_STATUS and write
 * NAND_REG_PROTECTION; both have dedicated LUT entries.
 */

static int sentai_nand_read_status(struct sentai_nand_dev_s *dev,
                                   uint8_t *val)
{
  uint32_t buf = 0;
  struct flexspi_transfer_s xfer =
  {
    .device_address = 0,
    .port           = dev->port,
    .cmd_type       = FLEXSPI_READ,
    .seq_index      = LUT_GET_FEATURE,
    .seq_number     = 1,
    .data           = &buf,
    .data_size      = 1,
  };

  int ret = FLEXSPI_TRANSFER(dev->flexspi, &xfer);
  if (ret == 0)
    {
      *val = (uint8_t)buf;
    }
  return ret;
}

static int sentai_nand_write_protect_reg(struct sentai_nand_dev_s *dev,
                                         uint8_t val)
{
  uint32_t buf = val;
  struct flexspi_transfer_s xfer =
  {
    .device_address = 0,
    .port           = dev->port,
    .cmd_type       = FLEXSPI_WRITE,
    .seq_index      = LUT_SET_FEATURE,
    .seq_number     = 1,
    .data           = &buf,
    .data_size      = 1,
  };

  return FLEXSPI_TRANSFER(dev->flexspi, &xfer);
}

static int sentai_nand_wait_busy(struct sentai_nand_dev_s *dev,
                                 unsigned timeout_us, uint8_t *out_status)
{
  uint8_t status;
  unsigned waited = 0;

  do
    {
      int ret = sentai_nand_read_status(dev, &status);
      if (ret < 0)
        {
          return ret;
        }

      if ((status & NAND_SR_BUSY) == 0)
        {
          if (out_status)
            {
              *out_status = status;
            }
          return 0;
        }

      up_udelay(50);
      waited += 50;
    }
  while (waited < timeout_us);

  return -ETIMEDOUT;
}

/****************************************************************************
 * NAND operations (page read / program, block erase)
 ****************************************************************************/

static int sentai_nand_read_page(struct sentai_nand_dev_s *dev,
                                 uint32_t page_index, uint8_t *buf)
{
  uint8_t status;
  uint32_t col = 0;
  int ret;

  /* Issue PAGE_READ (0x13) with the absolute page index as the row addr.
   * The chip latches the page from the array into its internal cache.
   */

  ret = sentai_nand_cmd(dev, LUT_PAGE_READ, page_index);
  if (ret < 0)
    {
      return ret;
    }

  ret = sentai_nand_wait_busy(dev, 500, &status);
  if (ret < 0)
    {
      return ret;
    }

  if ((status & NAND_SR_ECC_MASK) == NAND_SR_ECC_UNCORR)
    {
      ferr("ECC uncorrectable on page %u\n", (unsigned)page_index);
      return -EIO;
    }

  /* READ_FROM_CACHE (0x03) col=0, then 8 dummy cycles, then read 2048 B. */

  struct flexspi_transfer_s xfer =
  {
    .device_address = col,
    .port           = dev->port,
    .cmd_type       = FLEXSPI_READ,
    .seq_index      = LUT_READ_CACHE,
    .seq_number     = 2,
    .data           = (uint32_t *)buf,
    .data_size      = NAND_PAGE_DATA_SIZE,
  };

  return FLEXSPI_TRANSFER(dev->flexspi, &xfer);
}

static int sentai_nand_program_page(struct sentai_nand_dev_s *dev,
                                    uint32_t page_index,
                                    const uint8_t *buf)
{
  uint8_t status;
  int ret;

  ret = sentai_nand_cmd(dev, LUT_WRITE_ENABLE, 0);
  if (ret < 0)
    {
      return ret;
    }

  /* PROG_LOAD (0x02) col=0 + write 2048 B into chip cache. */

  struct flexspi_transfer_s load_xfer =
  {
    .device_address = 0,
    .port           = dev->port,
    .cmd_type       = FLEXSPI_WRITE,
    .seq_index      = LUT_PROG_LOAD,
    .seq_number     = 1,
    .data           = (uint32_t *)buf,
    .data_size      = NAND_PAGE_DATA_SIZE,
  };

  ret = FLEXSPI_TRANSFER(dev->flexspi, &load_xfer);
  if (ret < 0)
    {
      return ret;
    }

  /* PROG_EXECUTE (0x10) commits the cache to the array. */

  ret = sentai_nand_cmd(dev, LUT_PROG_EXEC, page_index);
  if (ret < 0)
    {
      return ret;
    }

  ret = sentai_nand_wait_busy(dev, 5000, &status);
  if (ret < 0)
    {
      return ret;
    }

  if (status & NAND_SR_PFAIL)
    {
      ferr("PROG_EXECUTE failed on page %u\n", (unsigned)page_index);
      return -EIO;
    }

  return 0;
}

static int sentai_nand_erase_block(struct sentai_nand_dev_s *dev,
                                   uint32_t block_index)
{
  uint8_t status;
  int ret;
  uint32_t row = block_index * NAND_PAGES_PER_BLOCK;

  ret = sentai_nand_cmd(dev, LUT_WRITE_ENABLE, 0);
  if (ret < 0)
    {
      return ret;
    }

  ret = sentai_nand_cmd(dev, LUT_BLOCK_ERASE, row);
  if (ret < 0)
    {
      return ret;
    }

  ret = sentai_nand_wait_busy(dev, 20000, &status);
  if (ret < 0)
    {
      return ret;
    }

  if (status & NAND_SR_EFAIL)
    {
      ferr("BLOCK_ERASE failed on block %u\n", (unsigned)block_index);
      return -EIO;
    }

  return 0;
}

/****************************************************************************
 * MTD interface
 ****************************************************************************/

static int sentai_nand_mtd_erase(struct mtd_dev_s *dev,
                                 off_t startblock, size_t nblocks)
{
  struct sentai_nand_dev_s *priv = (struct sentai_nand_dev_s *)dev;
  size_t i;
  int ret;

  for (i = 0; i < nblocks; i++)
    {
      ret = sentai_nand_erase_block(priv,
                                    NAND_USER_BLK_FIRST + startblock + i);
      if (ret < 0)
        {
          return ret;
        }
    }

  return nblocks;
}

static ssize_t sentai_nand_mtd_bread(struct mtd_dev_s *dev,
                                     off_t startblock,
                                     size_t nblocks,
                                     uint8_t *buffer)
{
  struct sentai_nand_dev_s *priv = (struct sentai_nand_dev_s *)dev;
  size_t i;
  int ret;

  /* Each MTD "block" here is one NAND ERASE block (64 pages). */

  for (i = 0; i < nblocks; i++)
    {
      uint32_t blk = NAND_USER_BLK_FIRST + startblock + i;
      uint32_t page;

      for (page = 0; page < NAND_PAGES_PER_BLOCK; page++)
        {
          ret = sentai_nand_read_page(priv,
                                      blk * NAND_PAGES_PER_BLOCK + page,
                                      buffer);
          if (ret < 0)
            {
              return ret;
            }

          buffer += NAND_PAGE_DATA_SIZE;
        }
    }

  return nblocks;
}

static ssize_t sentai_nand_mtd_bwrite(struct mtd_dev_s *dev,
                                      off_t startblock,
                                      size_t nblocks,
                                      const uint8_t *buffer)
{
  struct sentai_nand_dev_s *priv = (struct sentai_nand_dev_s *)dev;
  size_t i;
  int ret;

  for (i = 0; i < nblocks; i++)
    {
      uint32_t blk = NAND_USER_BLK_FIRST + startblock + i;
      uint32_t page;

      for (page = 0; page < NAND_PAGES_PER_BLOCK; page++)
        {
          ret = sentai_nand_program_page(priv,
                                         blk * NAND_PAGES_PER_BLOCK + page,
                                         buffer);
          if (ret < 0)
            {
              return ret;
            }

          buffer += NAND_PAGE_DATA_SIZE;
        }
    }

  return nblocks;
}

static int sentai_nand_mtd_ioctl(struct mtd_dev_s *dev, int cmd,
                                 unsigned long arg)
{
  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          struct mtd_geometry_s *geo = (struct mtd_geometry_s *)arg;
          if (geo == NULL)
            {
              return -EINVAL;
            }

          memset(geo, 0, sizeof(*geo));
          geo->blocksize    = NAND_PAGE_DATA_SIZE;
          geo->erasesize    = NAND_BLOCK_SIZE;
          geo->neraseblocks = NAND_USER_BLK_COUNT;
          return 0;
        }

      case BIOC_PARTINFO:
        {
          struct partition_info_s *info = (struct partition_info_s *)arg;
          if (info == NULL)
            {
              return -EINVAL;
            }

          info->numsectors  = NAND_USER_BLK_COUNT * NAND_PAGES_PER_BLOCK;
          info->sectorsize  = NAND_PAGE_DATA_SIZE;
          info->startsector = 0;
          info->parent[0]   = '\0';
          return 0;
        }

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Public init
 ****************************************************************************/

struct mtd_dev_s *imxrt_flexspi_nand_initialize(int intf)
{
  struct sentai_nand_dev_s *priv = &g_nand_dev;
  uint8_t status;
  int ret;

  if (priv->flexspi != NULL)
    {
      return &priv->mtd;
    }

  /* Configure the FlexSPI1 port-A pad mux for the on-board NAND.
   * Pin assignment matches coralmicro pin_mux.c (GPIO_SD_B2_06..11).
   * board.h's GPIO_FLEXSPI_* defines (option _2) point to the same
   * pads -- they were set up for the EVK NOR but the chip wired on
   * the Sentai board is the W25N NAND on the same FlexSPI1 port A.
   * imxrt_flexspi_initialize() will not configure these pins itself.
   */

  imxrt_config_gpio(GPIO_FLEXSPI1_A_SS0_B_1  | IOMUX_FLEXSPI_DEFAULT);
  imxrt_config_gpio(GPIO_FLEXSPI1_A_SCLK_1   | IOMUX_FLEXSPI_DEFAULT);
  imxrt_config_gpio(GPIO_FLEXSPI1_A_DATA0_1  | IOMUX_FLEXSPI_DEFAULT);
  imxrt_config_gpio(GPIO_FLEXSPI1_A_DATA1_1  | IOMUX_FLEXSPI_DEFAULT);
  imxrt_config_gpio(GPIO_FLEXSPI1_A_DATA2_1  | IOMUX_FLEXSPI_DEFAULT);
  imxrt_config_gpio(GPIO_FLEXSPI1_A_DATA3_1  | IOMUX_FLEXSPI_DEFAULT);

  priv->flexspi = imxrt_flexspi_initialize(intf);
  if (priv->flexspi == NULL)
    {
      ferr("imxrt_flexspi_initialize(%d) failed\n", intf);
      return NULL;
    }

  priv->port      = FLEXSPI_PORT_A1;
  priv->base_addr = 0;

  priv->mtd.erase  = sentai_nand_mtd_erase;
  priv->mtd.bread  = sentai_nand_mtd_bread;
  priv->mtd.bwrite = sentai_nand_mtd_bwrite;
  priv->mtd.ioctl  = sentai_nand_mtd_ioctl;

  /* Push our LUT into the controller. */

  FLEXSPI_UPDATE_LUT(priv->flexspi, 0, &g_nand_lut[0][0],
                     sizeof(g_nand_lut) / sizeof(uint32_t));
  FLEXSPI_SET_DEVICE_CONFIG(priv->flexspi,
                            (struct flexspi_device_config_s *)&g_nand_config,
                            priv->port);

  /* Reset and unlock all blocks (NAND ships with all-blocks-locked
   * write-protect bits set in NAND_REG_PROTECTION).
   */

  sentai_nand_cmd(priv, LUT_RESET, 0);
  up_mdelay(2);
  sentai_nand_wait_busy(priv, 5000, &status);

  /* Diagnostic: read JEDEC ID. Winbond W25N01: returns 0xEF 0xAA 0x21
   * (manufacturer / device hi / device lo). Used to confirm the LUT
   * + FlexSPI controller path is alive before we start exercising the
   * page/program/erase paths.
   */

  uint8_t id[4] = {0};
  struct flexspi_transfer_s id_xfer =
  {
    .device_address = 0,
    .port           = priv->port,
    .cmd_type       = FLEXSPI_READ,
    .seq_index      = LUT_READ_ID,
    .seq_number     = 1,
    .data           = (uint32_t *)id,
    .data_size      = 3,
  };

  if (FLEXSPI_TRANSFER(priv->flexspi, &id_xfer) == 0)
    {
      syslog(LOG_INFO, "[sentai-nand] JEDEC ID: %02x %02x %02x\n",
             id[0], id[1], id[2]);
    }
  else
    {
      syslog(LOG_ERR, "[sentai-nand] JEDEC ID read failed\n");
    }

  ret = sentai_nand_write_protect_reg(priv, 0x00);
  if (ret < 0)
    {
      ferr("nand: unlock failed: %d\n", ret);
      return NULL;
    }

  return &priv->mtd;
}

#endif /* CONFIG_SENTAI_RT1176_FLEXSPI_NAND */
