#!/usr/bin/env python
#
# Copyright 2010 Ettus Research LLC
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

########################################################################
# Template for raw text data describing registers
# name addr[bit range inclusive] default optional enums
########################################################################
REGS_TMPL="""\
########################################################################
## Register 0x000 SPI Configuration
########################################################################
spi_soft_reset                  0[7]        0       disabled, reset
spi_3wire                       0[6]        0       input, bidirectional
spi_lsb_first                   0[5]        0       msb, lsb
########################################################################
## Register 0x001 Multi-Chip Sync and Tx Mon Control
########################################################################
tx2_mon_enable                  1[6]        0       disable, enable
tx1_mon_enable                  1[5]        0       disable, enable
mcs_rf_enable                   1[3]        0       disable, enable
mcs_bbpll_enable                1[2]        0       disable, enable
mcs_digital_clks_enable         1[1]        0       disable, enable
mcs_bb_enable                   1[0]        0       disable, enable
########################################################################
## Register 0x002 Tx Enable & Filter Control
########################################################################
tx_channel_enable               2[6:7]      0       disabled, tx1, tx2, both_tx
thb3_interp_enable              2[4:5]      0       interp1_nofilt, interp2_hbfilt, interp3_filt
thb2_enable                     2[3]        0       disabled, enabled
thb1_enable                     2[2]        0       disabled, enabled
tx_fir_enable                   2[0:1]      0       interp1_nofilt, interp1_filt, interp2_filt, interp4_filt
########################################################################
## Register 0x003 Rx Enable & Filter Control
########################################################################
rx_channel_enable               3[6:7]      0       disabled, rx1, rx2, both_rx
rhb3_decim_enable               3[4:5]      0       decim1_nofilt, decim2_hbfilt, decim3_filt
rhb2_enable                     3[3]        0       disabled, enable
rhb1_enable                     3[2]        0       disabled, enable
rx_fir_enable                   3[0:1]      0       decim1_nofilt, decim1_filt, decim2_filt, decim4_filt
########################################################################
## Register 0x004 Input Select
########################################################################
"""

########################################################################
# Template for methods in the body of the struct
########################################################################
BODY_TMPL="""\
boost::uint8_t get_reg(boost::uint16_t addr){
    boost::uint8_t reg = 0;
    switch(addr){
    #for $addr in sorted(set(map(lambda r: r.get_addr(), $regs)))
    case $addr:
        #for $reg in filter(lambda r: r.get_addr() == addr, $regs)
        reg |= (boost::uint32_t($reg.get_name()) & $reg.get_mask()) << $reg.get_shift();
        #end for
        break;
    #end for
    }
    return reg;
}

boost::uint32_t get_write_reg(boost::uint16_t addr){
    return (boost::uint32_t(addr) << 8) | get_reg(addr);
}

boost::uint32_t get_read_reg(boost::uint16_t addr){
    return (boost::uint32_t(addr) << 8) | (1 << 23);
}
"""

if __name__ == '__main__':
    import common; common.generate(
        name='ad9361_regs',
        regs_tmpl=REGS_TMPL,
        body_tmpl=BODY_TMPL,
        file=__file__,
    )
