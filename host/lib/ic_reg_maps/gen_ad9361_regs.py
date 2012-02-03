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
spi_soft_reset              0x000[7]        0       disabled, reset
spi_3wire                   0x000[6]        0       input, bidirectional
spi_lsb_first               0x000[5]        0       msb, lsb
########################################################################
## Register 0x001 Multi-Chip Sync and Tx Mon Control
########################################################################
tx2_mon_enable              0x001[6]        0       disable, enable
tx1_mon_enable              0x001[5]        0       disable, enable
mcs_rf_enable               0x001[3]        0       disable, enable
mcs_bbpll_enable            0x001[2]        0       disable, enable
mcs_digital_clks_enable     0x001[1]        0       disable, enable
mcs_bb_enable               0x001[0]        0       disable, enable
########################################################################
## Register 0x002 Tx Enable & Filter Control
########################################################################
tx_channel_enable           0x002[6:7]      1       disabled, tx1, tx2, both_tx
thb3_interp_enable          0x002[4:5]      0       interp1_nofilt, interp2_hbfilt, interp3_filt
thb2_enable                 0x002[3]        1       disabled, enabled
thb1_enable                 0x002[2]        1       disabled, enabled
tx_fir_enable               0x002[0:1]      3       interp1_nofilt, interp1_filt, interp2_filt, interp4_filt
########################################################################
## Register 0x003 Rx Enable & Filter Control
########################################################################
rx_channel_enable           0x003[6:7]      1       disabled, rx1, rx2, both_rx
rhb3_decim_enable           0x003[4:5]      0       decim1_nofilt, decim2_hbfilt, decim3_filt
rhb2_enable                 0x003[3]        1       disabled, enable
rhb1_enable                 0x003[2]        1       disabled, enable
rx_fir_enable               0x003[0:1]      3       decim1_nofilt, decim1_filt, decim2_filt, decim4_filt
########################################################################
## Register 0x004 Input Select
########################################################################
reg_004_unused_D7           0x004[7]        0
tx_output                   0x004[6]        0       txa, txb
rx_input                    0x004[0:5]      0       rx1an_rx2an_en_unbal=1, rx1ap_rx2ap_en_unbal=2, rx1bn_rx2bn_en_unbal=4, rx1bp_rx2bp_en_unbal=8, rx1cn_rx2cn_en_unbal=16, rx1cp_rx2cp_en_unbal=32, rx1an_rx1ap_rx2an_rx2ap_en_bal=3, rx1bn_rx1bp_rx2bn_rx2bp_en_bal=12, rx1cn_rx1cp_rx2cn_rx2cp_en_bal=48
########################################################################
## Register 0x005 RFPLL Dividers
########################################################################
#set $vco_dividers = ', '.join(map(lambda x: 'div_by_' + str(2**(x + 1)) + '=' + str(x), range(0, 7))
#set $vco_dividers = $tx_vco_dividers + ', ext_vco_div_by_2=7'
tx_vco_divider              0x005[4:7]      0       $vco_dividers
rx_vco_divider              0x005[0:3]      0       $vco_dividers
########################################################################
## Register 0x006 Rx Clock and Data Delay
########################################################################
data_clk_delay              0x006[4:7]      0       0
rx_data_delay               0x006[0:3]      0       0
########################################################################
## Register 0x007 Tx Clock and Data Delay
########################################################################
fb_clk_delay                0x007[4:7]      0       0
tx_data_delay               0x007[0:3]      0       0
########################################################################
## Register 0x009 Clock Enable
########################################################################
bypass_bbpll                0x009[5]        0       disabled, enabled
xo_bypass                   0x009[4]        1       xtalp, xtaln
bbpll_force_lock            0x009[3]        0       disabled, enabled
digital_power_up            0x009[2]        0       disabled, enabled
dcxo_enable                 0x009[1]        0       disabled, enabled
bbpll_enable                0x009[0]        0       disabled, enabled
########################################################################
## Register 0x00A BBPLL
########################################################################
clkout                      0x00A[5:7]      0       ref_clk, adc_clk_div_2, adc_clk_div_3, adc_clk_div_4, adc_clk_div_8, adc_clk_div_16, adc_clk_div32, adc_clk_div64
clkout_enable               0x00A[4]        0       disabled, enabled
dac_clk_div2                0x00A[3]        0       disabled, enabled
bbpll_divider               0x00A[0:2]      3       divby1, divby2, divby4, divby8, divby16, divby32, divby64
########################################################################
## Register 0x00B Temp Sense Offset
########################################################################
temp_sensor_offset          0x00B[0:7]      0
########################################################################
## Register 0x00C Temp Sense 1
########################################################################
temp_window                 0x00C[4:7]      0
temp_window_exceed          0x00C[3]        0
temp_window_hilo            0x00C[2]        0
temp_valid                  0x00C[1]        0
start_temp_reading          0x00C[0]        0
########################################################################
## Register 0x00D Temp Sense 2
########################################################################
measurement_time_interval   0x00D[1:7]      1
temp_sense_periodic_enable  0x00D[0]        1       disabled, enabled
########################################################################
## Register 0x00E Temperature
########################################################################
tempurature_reading         0x00E[0:7]      0
########################################################################
## Register 0x00F Temp Sensor Config
########################################################################
enable_aux_adc_readback     0x00F[3]        1       disabled, enabled
temp_sensor_decimation      0x00F[0:2]      0
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
