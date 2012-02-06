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
rx_input                    0x004[0:5]      1       rx1an_rx2an_en_unbal=1, rx1ap_rx2ap_en_unbal=2, rx1bn_rx2bn_en_unbal=4, rx1bp_rx2bp_en_unbal=8, rx1cn_rx2cn_en_unbal=16, rx1cp_rx2cp_en_unbal=32, rx1an_rx1ap_rx2an_rx2ap_en_bal=3, rx1bn_rx1bp_rx2bn_rx2bp_en_bal=12, rx1cn_rx1cp_rx2cn_rx2cp_en_bal=48
########################################################################
## Register 0x005 RFPLL Dividers
########################################################################
#set $vco_dividers = ', '.join(map(lambda x: 'div_by_' + str(2**(x + 1)) + '=' + str(x), range(0, 7)))
#set $vco_dividers = $vco_dividers + ', ext_vco_div_by_2=7'
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
########################################################################
## Register 0x010 Parallel Port Configuration
########################################################################
pp_tx_swap_iq               0x010[7]        1       unswapped, swapped
pp_rx_swap_iq               0x010[6]        1       unswapped, swapped
tx_channel_swap             0x010[5]        0       unswapped, swapped
rx_channel_swap             0x010[4]        0       unswapped, swapped
rx_frame_pulse_mode         0x010[3]        0       level_mode, pulse_mode
timing_2t2r                 0x010[2]        0       disabled, enabled
invert_data_bus             0x010[1]        0       disabled, enabled
invert_data_clk             0x010[0]        0       disabled, enabled
########################################################################
## Register 0x011 Parallel Port Configuration 2
########################################################################
fdd_alt_word_order          0x011[7]        0       disabled, enabled
invert_rx1                  0x011[6]        0
invert_rx2                  0x011[5]        0
invert_tx1                  0x011[4]        0       disabled, enabled
invert_tx2                  0x011[3]        0       disabled, enabled
invert_rx_frame             0x011[2]        0       disabled, enabled
delay_rx_data               0x011[0:1]      0
########################################################################
## Register 0x012 Parallel Port Configuration 3
########################################################################
fdd_rx_rate                 0x012[7]        0       txrate, double_txrate
swap_ports                  0x012[6]        0       unswapped, swapped
single_data_rate            0x012[5]        0       ddr, sdr
lvds_mode                   0x012[4]        0       se_cmos, lvds
half_duplex_mode            0x012[3]        1       full_duplex, half_duplex
single_port_mode            0x012[2]        0       double, single
full_port                   0x012[1]        0       mixed, separate
full_duplex_swap_bits       0x012[0]        0       unswapped, swapped
########################################################################
## Register 0x013 ENSM Mode
########################################################################
fdd_mode                    0x013[7]        1       disabled, enabled
########################################################################
## Register 0x014 ENSM Config 1
########################################################################
always_enabled_rx_data_port 0x014[7]        0       disabled, enabled
force_rx_on                 0x014[6]        0       disabled, enabled
force_tx_on                 0x014[5]        0       disabled, enabled
enable_ensm_pin_ctrl        0x014[4]        1       signal_ctrl, spi_ctrl
level_mode                  0x014[3]        0       disabled, enabled
force_alert_state           0x014[2]        0       disabled, enabled
auto_gain_lock              0x014[1]        1       disabled, enabled
to_alert                    0x014[0]        1       to_wait, to_alert
########################################################################
## Register 0x015 ENSM Config 2
########################################################################
fdd_external_ctrl_enable    0x015[7]        0       disabled, enabled
power_down_rx_synth         0x015[6]        0       disabled, enabled
power_down_tx_synth         0x015[5]        0       disabled, enabled
txnrx_spi_ctrl              0x015[4]        0       disabled, enabled
synth_enable_pin_ctrl_mode  0x015[3]        1       disabled, enabled
dual_synth_mode             0x015[2]        0       one_synth, both_synths
rx_synth_ready_mask         0x015[1]        0       disabled, enabled
tx_synth_ready_mask         0x015[0]        0       disabled, enabled
########################################################################
## Register 0x016 Calibration Control
########################################################################
rx_bb_tune                  0x016[7]        0       complete, start
tx_bb_tune                  0x016[6]        0       complete, start
rx_quad_cal                 0x016[5]        0       complete, start
tx_quad_cal                 0x016[4]        0       complete, start
rx_gain_step_cal            0x016[3]        0       complete, start
tx_power_detect_cal         0x016[2]        0       complete, start
rf_dc_cal                   0x016[1]        0       complete, start
bb_dc_cal                   0x016[0]        0       complete, start
########################################################################
## Register 0x017 State
########################################################################
calibration_seq_state       0x017[4:7]      0
ensm_state                  0x017[0:3]      0
########################################################################
## Register 0x018, 0x019, 0x01A[0:1], 0x01B[0:1] AuxDAC 1,2 Words
########################################################################
auxdac1_word_msbits         0x018[0:7]      0
auxdac2_word_msbits         0x019[0:7]      0
auxdac1_word_lsbits         0x01A[0:1]      0
auxdac2_word_lsbits         0x01B[0:1]      0
########################################################################
## Register 0x01A AuxDAC1 Config
########################################################################
comp_ctrl1                  0x01A[5]        0
auxdac1_step_factor         0x01A[4]        0       step_2, step_1
auxdac1_vref                0x01A[2:3]      0       1V, 1_5V, 2V, 2_5V
########################################################################
## Register 0x01B AuxDAC2 Config
########################################################################
comp_ctrl2                  0x01B[5]        0
auxdac2_step_factor         0x01B[4]        0       step_2, step_1
auxdac2_vref                0x01B[2:3]      0       1V, 1_5V, 2V, 2_5V
########################################################################
## Register 0x01C AuxADC Clock Divider
########################################################################
auxadc_clock_divider        0x01C[0:5]      16
########################################################################
## Register 0x01D AuxADC Config
########################################################################
auxadc_decimation           0x01D[1:3]      0
auxadc_power_down           0x01D[0]        1       disabled, enabled
########################################################################
## Register 0x01E, 0x01F AuxADC Word
########################################################################
auxadc_word_msbits          0x01E[0:7]      0
auxadc_word_lsbits          0x01F[0:3]      0
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
