/*
 * chinch.c
 *
 *  Created on: May 14, 2013
 *      Author: ashish
 */

#include "chinch.h"

#define CHINCH_REG_BASE                 0xA000

#define CHINCH_CONTROL_REG_WRITE        (1<<30)
#define CHINCH_CONTROL_REG_READ         (1<<29)
#define CHINCH_CONTROL_REG_HALF_WORD    (1<<28)
#define CHINCH_CONTROL_REG_ADDR_MASK    0x000FFFFF

#define CHINCH_STATUS_REG_REQ_PENDING   (1<<1)
#define CHINCH_STATUS_REG_READ_PENDING  (1<<0)
#define CHINCH_STATUS_REG_PENDING_MASK  (CHINCH_STATUS_REG_REQ_PENDING|CHINCH_STATUS_REG_READ_PENDING)

#define CHINCH_SIG_REG                  0x0
#define CHINCH_SCRATCH_REG_BASE         0x200
#define CHINCH_CPWBS_REG                0xC0
#define CHINCH_CPWC_REG                 0xE0
#define CHINCH_FLASH_2AAA_REG           0x400
#define CHINCH_FLASH_5555_REG           0x408
#define CHINCH_FLASH_WINDOW_BASE        0xC0000

//-----------------------------------------------------
// Peek-Poke interface
//-----------------------------------------------------

bool chinch_poke(
    const uint32_t addr,
    const uint32_t data,
    bool half_word,
    uint32_t timeout
)
{
    //Build transaction control word
    uint32_t ctrl_word = 0, i;
    ctrl_word |= (addr & CHINCH_CONTROL_REG_ADDR_MASK);
    if (half_word) ctrl_word |= CHINCH_CONTROL_REG_HALF_WORD;
    ctrl_word |= CHINCH_CONTROL_REG_WRITE;

    //Wait for space in the transaction queue or timeout
    i = 0;
    while ((wb_peek32(SR_ADDR(CHINCH_REG_BASE, CHINCH_STATUS_REG)) & CHINCH_STATUS_REG_PENDING_MASK) != 0) {
        if (++i > timeout) return false;
    }

    //Flush transaction control and data registers
    wb_poke32(SR_ADDR(CHINCH_REG_BASE, CHINCH_DATA_WR_REG), data);
    wb_poke32(SR_ADDR(CHINCH_REG_BASE, CHINCH_CONTROL_REG), ctrl_word);

    return true;
}

bool chinch_peek(
    const uint32_t addr,
    uint32_t* data,
    bool half_word,
    uint32_t timeout
)
{
    //Build transaction control word
    uint32_t ctrl_word = 0, i;
    ctrl_word |= (addr & CHINCH_CONTROL_REG_ADDR_MASK);
    if (half_word) ctrl_word |= CHINCH_CONTROL_REG_HALF_WORD;
    ctrl_word |= CHINCH_CONTROL_REG_READ;

    //Wait for space in the transaction queue or timeout
    i = 0;
    while ((wb_peek32(SR_ADDR(CHINCH_REG_BASE, CHINCH_STATUS_REG)) & CHINCH_STATUS_REG_PENDING_MASK) != 0) {
        if (++i > timeout) return false;
    }

    //Flush transaction control register
    if (data) *data = 0;
    wb_poke32(SR_ADDR(CHINCH_REG_BASE, CHINCH_CONTROL_REG), ctrl_word);

    //Wait for read completion or timeout
    i = 0;
    while ((wb_peek32(SR_ADDR(CHINCH_REG_BASE, CHINCH_STATUS_REG)) & CHINCH_STATUS_REG_READ_PENDING) != 0) {
        if (++i > timeout) return false;
    }

    //Read transaction data register
    if (data) *data = wb_peek32(SR_ADDR(CHINCH_REG_BASE, CHINCH_DATA_RD_REG));
    return true;
}

//-----------------------------------------------------
// Flash access
//-----------------------------------------------------

uint32_t    g_cached_cpwbsr;
uint32_t    g_cached_cpwcr;
int32_t     g_writes_remaining;

bool chinch_flash_init()
{
    //Backup window and page registers
    chinch_peek32(CHINCH_CPWBS_REG, &g_cached_cpwbsr);
    chinch_peek32(CHINCH_CPWC_REG, &g_cached_cpwcr);

    bool status = true;
    //Setup window
    STATUS_CHAIN(chinch_poke32(CHINCH_CPWBS_REG, CHINCH_FLASH_WINDOW_BASE | 0x92), status);

    //Run a loopback test to ensure that we will not corrupt the flash.
    STATUS_CHAIN(chinch_poke32(CHINCH_SCRATCH_REG_BASE + 0, 0xDEADBEEF), status);
    STATUS_CHAIN(chinch_poke16(CHINCH_SCRATCH_REG_BASE + 4, 0x5678), status);
    uint32_t reg_val;
    STATUS_CHAIN(chinch_peek16(CHINCH_SIG_REG, &reg_val), status);
    STATUS_CHAIN(chinch_poke16(CHINCH_SCRATCH_REG_BASE + 6, reg_val), status);
    STATUS_CHAIN(chinch_peek32(CHINCH_SCRATCH_REG_BASE + 0, &reg_val), status);

    return status && (reg_val == 0xDEADBEEF);
}

void chinch_flash_cleanup()
{
    //Restore window and page registers
    chinch_poke32(CHINCH_CPWBS_REG, g_cached_cpwbsr);
    chinch_poke32(CHINCH_CPWC_REG, g_cached_cpwcr);
}

bool chinch_flash_select_sector(uint32_t sector)
{
    return chinch_poke32(CHINCH_CPWC_REG, sector << 17);
}

bool chinch_flash_erase_sector()
{
    bool status = true;
    STATUS_CHAIN(chinch_poke16(CHINCH_FLASH_5555_REG,    0x00AA), status);    //Unlock #1
    STATUS_CHAIN(chinch_poke16(CHINCH_FLASH_2AAA_REG,    0x0055), status);    //Unlock #2
    STATUS_CHAIN(chinch_poke16(CHINCH_FLASH_5555_REG,    0x0080), status);    //Setup
    STATUS_CHAIN(chinch_poke16(CHINCH_FLASH_5555_REG,    0x00AA), status);    //Unlock #1
    STATUS_CHAIN(chinch_poke16(CHINCH_FLASH_2AAA_REG,    0x0055), status);    //Unlock #2
    STATUS_CHAIN(chinch_poke16(CHINCH_FLASH_WINDOW_BASE, 0x0030), status);    //Erase

    if (status) {
        uint32_t read_data;
        while (true) {
            status = chinch_peek16(CHINCH_FLASH_WINDOW_BASE, &read_data);    //Wait for sector to erase
            if (((read_data & 0xFFFF) == 0xFFFF) || !status) break;
        }
    }
    return status;
}

bool chinch_flash_write_start(uint32_t num_hwords)
{
    if (num_hwords > 32) return false;

    bool status = true;
    STATUS_CHAIN(chinch_poke16(CHINCH_FLASH_5555_REG,    0x00AA), status);       //Unlock #1
    STATUS_CHAIN(chinch_poke16(CHINCH_FLASH_2AAA_REG,    0x0055), status);       //Unlock #2
    STATUS_CHAIN(chinch_poke16(CHINCH_FLASH_WINDOW_BASE, 0x0025), status);       //Setup write
    STATUS_CHAIN(chinch_poke16(CHINCH_FLASH_WINDOW_BASE, num_hwords-1), status); //Num words

    g_writes_remaining = status ? num_hwords : 0;
    return status;
}

bool chinch_flash_write_commit()
{
    return chinch_poke16(CHINCH_FLASH_WINDOW_BASE, 0x0029);
}

bool chinch_flash_write(uint32_t offset, uint32_t hword)
{
    if (g_writes_remaining-- < 1) return false;
    return chinch_poke16(CHINCH_FLASH_WINDOW_BASE | (offset & 0x3FFFF), hword);
}

bool chinch_flash_read(uint32_t offset, uint32_t* hword)
{
    return chinch_peek16(CHINCH_FLASH_WINDOW_BASE | (offset & 0x3FFFF), hword);
}
