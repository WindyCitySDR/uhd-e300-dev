/*
 * chinch.c
 *
 *  Created on: May 14, 2013
 *      Author: ashish
 */

#include "chinch.h"

#define PCIE_MSG_REG_BASE   0xFB00
#define PCIE_MSG_DATA_REG   0 //Write: Data register for outbound requests and responses
                              //Read:  Latched data for inbound requests
#define PCIE_MSG_CTRL_REG   1 //Write: Control register for outbound requests and responses (Initiates xfer)
                              //Read:  Latched control word for inbound requests
#define PCIE_MSG_RESP_REG   2 //Read:  Latched response data for outbound requests
#define PCIE_MSG_STATUS_REG 3 //Read:  Status register for inbound and outbound transactions

// Transaction Word Format
//
// -Control- --Data--
// 63     32 31      0
// |       | |       |
// FXXA AAAA DDDD DDDD
//
// where:
// D = Data/Payload
// A = Address
// X = Reserved
// F = Flags: {Read_Response, Write_Request, Read_Request, Half_Word_Cycle}

#define PCIE_CTRL_REG_READ_RESP         (1<<31)
#define PCIE_CTRL_REG_WRITE             (1<<30)
#define PCIE_CTRL_REG_READ              (1<<29)
#define PCIE_CTRL_REG_HALF_WORD         (1<<28)
#define PCIE_CTRL_REG_ADDR_MASK         0x000FFFFF

#define PCIE_STATUS_REG_READ_PENDING    (1<<0)
#define PCIE_STATUS_REG_REQ_PENDING     (1<<1)
#define PCIE_STATUS_REG_RESP_PENDING    (1<<2)
#define PCIE_STATUS_REG_BUSY            (1<<4)

#define CHINCH_SIG_REG                  0x0
#define CHINCH_CCCS_REG                 0x58
#define CHINCH_CPWBS_REG                0xC0
#define CHINCH_CPWC_REG                 0xE0
#define CHINCH_SCRATCH_REG_BASE         0x200
#define CHINCH_FLASH_2AAA_REG           0x400
#define CHINCH_FLASH_5555_REG           0x408
#define CHINCH_FLASH_WINDOW_BASE        0x60000
#define CHINCH_FLASH_WINDOW_SIZE        0x20000
#define CHINCH_FLASH_WINDOW_CONF        0x91    //ESSS SSSS: Enable, Size (as pow of 2)

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
    ctrl_word |= (addr & PCIE_CTRL_REG_ADDR_MASK);
    if (half_word) ctrl_word |= PCIE_CTRL_REG_HALF_WORD;
    ctrl_word |= PCIE_CTRL_REG_WRITE;

    //Wait for space in the transaction queue or timeout
    i = 0;
    while ((wb_peek32(SR_ADDR(PCIE_MSG_REG_BASE, PCIE_MSG_STATUS_REG)) & PCIE_STATUS_REG_BUSY) != 0) {
        if (++i > timeout) return false;
    }

    //Flush transaction control and data registers
    wb_poke32(SR_ADDR(PCIE_MSG_REG_BASE, PCIE_MSG_DATA_REG), data);
    wb_poke32(SR_ADDR(PCIE_MSG_REG_BASE, PCIE_MSG_CTRL_REG), ctrl_word);

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
    ctrl_word |= (addr & PCIE_CTRL_REG_ADDR_MASK);
    if (half_word) ctrl_word |= PCIE_CTRL_REG_HALF_WORD;
    ctrl_word |= PCIE_CTRL_REG_READ;

    //Wait for space in the transaction queue or timeout
    i = 0;
    while ((wb_peek32(SR_ADDR(PCIE_MSG_REG_BASE, PCIE_MSG_STATUS_REG)) & PCIE_STATUS_REG_BUSY) != 0) {
        if (++i > timeout) return false;
    }

    //Flush transaction control register
    if (data) *data = 0;
    wb_poke32(SR_ADDR(PCIE_MSG_REG_BASE, PCIE_MSG_CTRL_REG), ctrl_word);

    //Wait for read completion or timeout
    i = 0;
    while ((wb_peek32(SR_ADDR(PCIE_MSG_REG_BASE, PCIE_MSG_STATUS_REG)) & PCIE_STATUS_REG_READ_PENDING) != 0) {
        if (++i > timeout) return false;
    }
    //Read transaction data register
    if (data) *data = wb_peek32(SR_ADDR(PCIE_MSG_REG_BASE, PCIE_MSG_RESP_REG));
    return true;
}

//-----------------------------------------------------
// Flash access
//-----------------------------------------------------

uint32_t    g_cached_cpwbsr;
uint32_t    g_cached_cpwcr;

bool chinch_flash_init()
{
    //Backup window and page registers
    chinch_peek32(CHINCH_CPWBS_REG, &g_cached_cpwbsr);
    chinch_peek32(CHINCH_CPWC_REG, &g_cached_cpwcr);

    bool status = true, passed = true;
    //Setup window
    STATUS_MERGE(chinch_poke32(CHINCH_CPWBS_REG, CHINCH_FLASH_WINDOW_BASE | CHINCH_FLASH_WINDOW_CONF), status);

    //Run a loopback test to ensure that we will not corrupt the flash.
    STATUS_MERGE(chinch_poke32(CHINCH_SCRATCH_REG_BASE + 0, 0xDEADBEEF), status);
    STATUS_MERGE(chinch_poke16(CHINCH_SCRATCH_REG_BASE + 4, 0x5678), status);
    uint32_t reg_val;
    STATUS_MERGE(chinch_peek16(CHINCH_SIG_REG, &reg_val), status);
    STATUS_MERGE(chinch_poke16(CHINCH_SCRATCH_REG_BASE + 6, reg_val), status);
    STATUS_MERGE(chinch_peek32(CHINCH_SCRATCH_REG_BASE + 0, &reg_val), status);
    passed &= (reg_val == 0xDEADBEEF);
    STATUS_MERGE(chinch_peek32(CHINCH_SCRATCH_REG_BASE + 4, &reg_val), status);
    passed &= (reg_val == 0x7AD05678);

    return status && passed;
}

void chinch_flash_cleanup()
{
    //Restore window and page registers
    chinch_poke32(CHINCH_CPWBS_REG, g_cached_cpwbsr);
    chinch_poke32(CHINCH_CPWC_REG, g_cached_cpwcr);
}

bool chinch_flash_select_sector(uint32_t sector)
{
    return chinch_poke32(CHINCH_CPWC_REG, sector * CHINCH_FLASH_WINDOW_SIZE);
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

bool chinch_flash_read_buf(uint32_t offset, uint16_t* buf, uint32_t size)
{
    bool status = true;
    uint32_t base_addr = CHINCH_FLASH_WINDOW_BASE | (offset & 0x3FFFF);
    for (uint32_t i = 0; (i < size) && status; i++) {
        uint32_t word;
        STATUS_CHAIN(chinch_peek16(base_addr + (i * 2), &word), status);
        buf[i] = (uint16_t)word;
    }
    return status;
}

bool chinch_flash_write_buf(uint32_t offset, uint16_t* buf, uint32_t size)
{
    if (size > CHINCH_FLASH_MAX_BUF_WRITES || buf == 0) return false;
    bool status = true;

    //Setup buffered write
    STATUS_CHAIN(chinch_poke16(CHINCH_FLASH_5555_REG,    0x00AA), status);      //Unlock #1
    STATUS_MERGE(chinch_poke16(CHINCH_FLASH_2AAA_REG,    0x0055), status);      //Unlock #2
    STATUS_CHAIN(chinch_poke16(CHINCH_FLASH_WINDOW_BASE, 0x0025), status);      //Setup write
    STATUS_CHAIN(chinch_poke16(CHINCH_FLASH_WINDOW_BASE, size - 1), status);    //Num words

    //Write the data
    uint32_t base_addr = CHINCH_FLASH_WINDOW_BASE | (offset & 0x3FFFF);
    for (uint32_t i = 0; i < size; i++) {
        STATUS_CHAIN(chinch_poke16(base_addr + (i * 2), buf[i]), status);
    }

    //Commit write and poll until data is written
    STATUS_CHAIN(chinch_poke16(CHINCH_FLASH_WINDOW_BASE, 0x0029), status);
    if (status) {
        uint16_t read_data;
        while (true) {
            STATUS_MERGE(chinch_flash_read(base_addr, &read_data), status);    //Wait for write to finish
            if ((read_data == buf[0]) || !status) break;
        }
    }
    return status;
}

//-----------------------------------------------------
// POSC
//-----------------------------------------------------
void chinch_start_posc()
{
    chinch_poke32(CHINCH_CCCS_REG, 0x1);
}

chinch_posc_status_t chinch_get_posc_status()
{
    bool status = true;
    uint32_t read_data;
    STATUS_MERGE(chinch_peek32(CHINCH_CCCS_REG, &read_data), status);
    return status ? (chinch_posc_status_t)read_data : CHINCH_POSC_ERROR;
}

//-----------------------------------------------------
// Read-back interface for the user initiated
// PCIe register transactions
//-----------------------------------------------------
pcie_register_xact_t    g_pcie_reg_xact_info;
uint32_t                g_pcie_res_timeout;

bool _respond_to_pcie_xact_request(uint32_t response, uint32_t timeout)
{
    //Wait for space in the transaction queue or timeout
    uint32_t i = 0;
    while ((wb_peek32(SR_ADDR(PCIE_MSG_REG_BASE, PCIE_MSG_STATUS_REG)) & PCIE_STATUS_REG_BUSY) != 0) {
        if (++i > g_pcie_res_timeout) return false;
    }

    //First write data and then the control register to ensure coherency
    wb_poke32(SR_ADDR(PCIE_MSG_REG_BASE, PCIE_MSG_DATA_REG), response);
    wb_poke32(SR_ADDR(PCIE_MSG_REG_BASE, PCIE_MSG_CTRL_REG), PCIE_CTRL_REG_READ_RESP);

    return true;
}

bool check_pcie_user_regport(pcie_register_xact_t** xact_info_hdl)
{
    //Check for pending transaction requests
    if ((wb_peek32(SR_ADDR(PCIE_MSG_REG_BASE, PCIE_MSG_STATUS_REG)) & PCIE_STATUS_REG_REQ_PENDING) != 0) {
        //Attach responder to transaction info
        g_pcie_reg_xact_info.respond = _respond_to_pcie_xact_request;

        //First read data and then the control register to ensure coherency
        g_pcie_reg_xact_info.data = wb_peek32(SR_ADDR(PCIE_MSG_REG_BASE, PCIE_MSG_DATA_REG));
        uint32_t xact_control = wb_peek32(SR_ADDR(PCIE_MSG_REG_BASE, PCIE_MSG_CTRL_REG));

        g_pcie_reg_xact_info.addr = xact_control & PCIE_CTRL_REG_ADDR_MASK;
        g_pcie_reg_xact_info.size =
            (xact_control & PCIE_CTRL_REG_HALF_WORD) == 0 ? PCIE_XACT_32_BIT : PCIE_XACT_16_BIT;
        if ((xact_control & PCIE_CTRL_REG_READ) != 0)
            g_pcie_reg_xact_info.type = PCIE_XACT_READ;
        else if ((xact_control & PCIE_CTRL_REG_WRITE) != 0)
            g_pcie_reg_xact_info.type = PCIE_XACT_WRITE;
        else
            g_pcie_reg_xact_info.type = PCIE_XACT_ERROR;

        *xact_info_hdl = &g_pcie_reg_xact_info;
        return true;
    } else {
        *xact_info_hdl = 0;
        return false;
    }
}

bool forward_pcie_user_xact_to_wb()
{
    pcie_register_xact_t* xact_info;
    if (check_pcie_user_regport(&xact_info)) {
        if (xact_info->size == PCIE_XACT_32_BIT) {
            //Only respond to 32-bit transactions because that is all the LVFPGA interface can send
            if (xact_info->type == PCIE_XACT_WRITE) {
                wb_poke32(xact_info->addr, xact_info->data);
                return true;
            } else if (xact_info->type == PCIE_XACT_READ) {
                return xact_info->respond(wb_peek32(xact_info->addr), CHINCH_DEFAULT_XACT_TIMEOUT);
            }
        }
    }
    return false;
}



