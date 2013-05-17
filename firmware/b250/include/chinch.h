/*
 * chinch.h
 *
 *  Created on: May 14, 2013
 *      Author: ashish
 */

#ifndef INCLUDED_CHINCH_H
#define INCLUDED_CHINCH_H

#include <wb_utils.h>
#include <stdbool.h>
#include <printf.h>

static const uint32_t CHINCH_DATA_WR_REG = 128;
static const uint32_t CHINCH_CONTROL_REG = 129;
static const uint32_t CHINCH_DATA_RD_REG = 5;
static const uint32_t CHINCH_STATUS_REG  = 6;

static const uint32_t CHINCH_DEFAULT_XACT_TIMEOUT  = 32768;

#define STATUS_CHAIN(x, status) if (status) status = (x)
#define STATUS_CHAIN_DBG(x, status) STATUS_CHAIN(x, status); printf("%s: %s\n\r", #x, status?"succeeded":"failed!")

//-----------------------------------------------------
// Peek-Poke interface
//-----------------------------------------------------
bool chinch_poke(const uint32_t addr, const uint32_t data, bool half_word, uint32_t timeout);
bool chinch_peek(const uint32_t addr, uint32_t* data, bool half_word, uint32_t timeout);

static inline bool chinch_poke32(const uint32_t addr, const uint32_t data) {
    return chinch_poke(addr, data, false /*half word*/, CHINCH_DEFAULT_XACT_TIMEOUT);
}

static inline bool chinch_poke16(const uint32_t addr, const uint32_t data) {
    return chinch_poke(addr, data, true /*half word*/, CHINCH_DEFAULT_XACT_TIMEOUT);
}

static inline bool chinch_peek32(const uint32_t addr, uint32_t* data) {
    return chinch_peek(addr, data, false /*half word*/, CHINCH_DEFAULT_XACT_TIMEOUT);
}

static inline bool chinch_peek16(const uint32_t addr, uint32_t* data) {
    return chinch_peek(addr, data, true /*half word*/, CHINCH_DEFAULT_XACT_TIMEOUT);
}

//-----------------------------------------------------
// Flash access
//-----------------------------------------------------
bool chinch_flash_init();
void chinch_flash_cleanup();
bool chinch_flash_select_sector(uint32_t sector);
bool chinch_flash_erase_sector();
bool chinch_flash_write_start(uint32_t num_hwords);
bool chinch_flash_write_commit();
bool chinch_flash_write(uint32_t offset, uint32_t hword);
bool chinch_flash_read(uint32_t offset, uint32_t* hword);

#endif /* INCLUDED_CHINCH_H */
