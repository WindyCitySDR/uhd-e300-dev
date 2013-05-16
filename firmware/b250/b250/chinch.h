/*
 * chinch.h
 *
 *  Created on: May 14, 2013
 *      Author: ashish
 */

#ifndef INCLUDED_CHINCH_H
#define INCLUDED_CHINCH_H

#include <wb_utils.h>

static const uint32_t CHINCH_DATA_WR_REG = 128;
static const uint32_t CHINCH_CONTROL_REG = 129;
static const uint32_t CHINCH_DATA_RD_REG = 5;
static const uint32_t CHINCH_STATUS_REG  = 6;
static const uint32_t DEFAULT_TIMEOUT    = 32768;

#define CHINCH_REG_BASE                    0xA000

#define CHINCH_CONTROL_REG_WRITE        (1<<30)
#define CHINCH_CONTROL_REG_READ         (1<<29)
#define CHINCH_CONTROL_REG_HALF_WORD    (1<<28)
#define CHINCH_CONTROL_REG_ADDR_MASK    0x000FFFFF

#define CHINCH_STATUS_REG_REQ_PENDING   (1<<1)
#define CHINCH_STATUS_REG_READ_PENDING  (1<<0)
#define CHINCH_STATUS_REG_PENDING_MASK  (CHINCH_STATUS_REG_REQ_PENDING|CHINCH_STATUS_REG_READ_PENDING)

#define STATUS_CHAIN(x, status) if (status) status = (x)

//-----------------------------------------------------
// Peek-Poke interface
//-----------------------------------------------------

static inline bool chinch_poke(
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

static inline bool chinch_peek(
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

    //Flush transaction control registers
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

static inline bool chinch_poke32(const uint32_t addr, const uint32_t data)
{
    return chinch_poke(addr, data, false /*half word*/, DEFAULT_TIMEOUT);
}

static inline bool chinch_poke16(const uint32_t addr, const uint32_t data)
{
    return chinch_poke(addr, data, true /*half word*/, DEFAULT_TIMEOUT);
}

static inline bool chinch_peek32(const uint32_t addr, uint32_t* data)
{
    return chinch_peek(addr, data, false /*half word*/, DEFAULT_TIMEOUT);
}

static inline bool chinch_peek16(const uint32_t addr, uint32_t* data)
{
    return chinch_peek(addr, data, true /*half word*/, DEFAULT_TIMEOUT);
}

//-----------------------------------------------------
// Flash access
//-----------------------------------------------------

typedef struct
{
    uint32_t cpwbsr;
    uint32_t cpwcr;
} chinch_flash_config_t;

static inline bool chinch_flash_init(chinch_flash_config_t* restore_data)
{
    //Backup window and page registers
    chinch_peek32(0xC0, &(restore_data->cpwbsr));
    chinch_peek32(0xE0, &(restore_data->cpwbsr));

    bool status = true;
    //Setup window
    STATUS_CHAIN(chinch_poke32(0xC0, 0x000C0092), status);

    //Run a loopback test to ensure that we will not corrupt the flash.
    STATUS_CHAIN(chinch_poke32(0x200, 0xDEADBEEF), status);
    STATUS_CHAIN(chinch_poke16(0x204, 0x5678), status);
    uint32_t reg_val;
    STATUS_CHAIN(chinch_peek16(0x0, &reg_val), status);
    STATUS_CHAIN(chinch_poke16(0x206, reg_val), status);
    STATUS_CHAIN(chinch_peek32(0x200, &reg_val), status);

    return status && (reg_val == 0xDEADBEEF);
}

static inline void chinch_flash_cleanup(chinch_flash_config_t* restore_data)
{
    //Restore window and page registers
    chinch_poke32(0xC0, restore_data->cpwbsr);
    chinch_poke32(0xE0, restore_data->cpwbsr);
}

static inline bool chinch_flash_select_sector(uint32_t sector)
{
    return chinch_poke32(0x000E0, sector << 17);
}

static inline bool chinch_flash_erase_sector()
{
    bool status = true;
    STATUS_CHAIN(chinch_poke16(0x00408, 0x00AA), status);    //Unlock #1
    STATUS_CHAIN(chinch_poke16(0x00400, 0x0055), status);    //Unlock #2
    STATUS_CHAIN(chinch_poke16(0x00408, 0x0080), status);    //Setup
    STATUS_CHAIN(chinch_poke16(0x00408, 0x00AA), status);    //Unlock #1
    STATUS_CHAIN(chinch_poke16(0x00400, 0x0055), status);    //Unlock #2
    STATUS_CHAIN(chinch_poke16(0xC0000, 0x0030), status);    //Erase

    if (status) {
        uint32_t read_data;
        while (true) {
            status = chinch_peek16(0xC0000, &read_data);    //Wait for sector to erase
            if (((read_data & 0xFFFF) == 0xFFFF) || !status) break;
        }
    }
    return status;
}

static inline bool chinch_flash_write_prep(uint32_t num_hwords)
{
    bool status = true;
    STATUS_CHAIN(chinch_poke16(0x00408, 0x00AA), status);        //Unlock #1
    STATUS_CHAIN(chinch_poke16(0x00400, 0x0055), status);        //Unlock #2
    STATUS_CHAIN(chinch_poke16(0xC0000, 0x0025), status);        //Setup write
    STATUS_CHAIN(chinch_poke16(0xC0000, num_hwords-1), status); //Num words
    return status;
}

static inline bool chinch_flash_write_commit()
{
    return chinch_poke16(0xC0000, 0x0029);
}

static inline bool chinch_flash_write(uint32_t offset, uint32_t hword)
{
    return chinch_poke16(0xC0000 + (offset & 0x3FFFF), hword);
}

static inline bool chinch_flash_read(uint32_t offset, uint32_t* hword)
{

return chinch_peek16(0xC0000 + (offset & 0x3FFFF), hword);
}


#endif /* INCLUDED_CHINCH_H */
