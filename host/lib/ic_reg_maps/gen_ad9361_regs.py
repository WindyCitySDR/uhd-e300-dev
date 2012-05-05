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
########################################################################
## Register 0x020 Auto GPO
########################################################################
gpo_enable_auto_rx          0x020[4:7]      3
gpo_enable_auto_tx          0x020[0:3]      3
########################################################################
## Auto-magically generated
########################################################################
unnamed_reg_020         0x020[0:7]      51
unnamed_reg_021         0x021[0:7]      10
unnamed_reg_022         0x022[0:7]      10
unnamed_reg_023         0x023[0:7]      63
unnamed_reg_024         0x024[0:7]      2
unnamed_reg_025         0x025[0:7]      2
unnamed_reg_026         0x026[0:7]      0
unnamed_reg_027         0x027[0:7]      3
unnamed_reg_028         0x028[0:7]      0
unnamed_reg_029         0x029[0:7]      0
unnamed_reg_02a         0x02a[0:7]      0
unnamed_reg_02b         0x02b[0:7]      0
unnamed_reg_02c         0x02c[0:7]      0
unnamed_reg_02d         0x02d[0:7]      0
unnamed_reg_02e         0x02e[0:7]      0
unnamed_reg_02f         0x02f[0:7]      0
unnamed_reg_030         0x030[0:7]      0
unnamed_reg_031         0x031[0:7]      0
unnamed_reg_032         0x032[0:7]      0
unnamed_reg_033         0x033[0:7]      0
unnamed_reg_035         0x035[0:7]      0
unnamed_reg_036         0x036[0:7]      255
unnamed_reg_037         0x037[0:7]      0
unnamed_reg_038         0x038[0:7]      128
unnamed_reg_03a         0x03a[0:7]      0
unnamed_reg_03b         0x03b[0:7]      0
unnamed_reg_03c         0x03c[0:7]      3
unnamed_reg_03d         0x03d[0:7]      0
unnamed_reg_03e         0x03e[0:7]      0
unnamed_reg_03f         0x03f[0:7]      1
unnamed_reg_040         0x040[0:7]      0
unnamed_reg_041         0x041[0:7]      0
unnamed_reg_042         0x042[0:7]      0
unnamed_reg_043         0x043[0:7]      0
unnamed_reg_044         0x044[0:7]      16
unnamed_reg_045         0x045[0:7]      0
unnamed_reg_046         0x046[0:7]      9
unnamed_reg_047         0x047[0:7]      0
unnamed_reg_048         0x048[0:7]      197
unnamed_reg_049         0x049[0:7]      184
unnamed_reg_04a         0x04a[0:7]      46
unnamed_reg_04b         0x04b[0:7]      192
unnamed_reg_04c         0x04c[0:7]      0
unnamed_reg_04d         0x04d[0:7]      0
unnamed_reg_04e         0x04e[0:7]      0
unnamed_reg_04f         0x04f[0:7]      0
unnamed_reg_050         0x050[0:7]      0
unnamed_reg_051         0x051[0:7]      0
unnamed_reg_052         0x052[0:7]      3
unnamed_reg_053         0x053[0:7]      0
unnamed_reg_054         0x054[0:7]      0
unnamed_reg_055         0x055[0:7]      0
unnamed_reg_056         0x056[0:7]      0
unnamed_reg_057         0x057[0:7]      60
unnamed_reg_058         0x058[0:7]      48
unnamed_reg_05e         0x05e[0:7]      0
unnamed_reg_05f         0x05f[0:7]      0
unnamed_reg_060         0x060[0:7]      0
unnamed_reg_061         0x061[0:7]      0
unnamed_reg_062         0x062[0:7]      0
unnamed_reg_063         0x063[0:7]      0
unnamed_reg_064         0x064[0:7]      0
unnamed_reg_065         0x065[0:7]      0
unnamed_reg_067         0x067[0:7]      19
unnamed_reg_068         0x068[0:7]      24
unnamed_reg_069         0x069[0:7]      0
unnamed_reg_06a         0x06a[0:7]      0
unnamed_reg_06b         0x06b[0:7]      0
unnamed_reg_06c         0x06c[0:7]      0
unnamed_reg_06d         0x06d[0:7]      0
unnamed_reg_06e         0x06e[0:7]      8
unnamed_reg_06f         0x06f[0:7]      0
unnamed_reg_070         0x070[0:7]      193
unnamed_reg_071         0x071[0:7]      193
unnamed_reg_073         0x073[0:7]      0
unnamed_reg_074         0x074[0:7]      0
unnamed_reg_075         0x075[0:7]      0
unnamed_reg_076         0x076[0:7]      0
unnamed_reg_077         0x077[0:7]      64
unnamed_reg_078         0x078[0:7]      60
unnamed_reg_079         0x079[0:7]      0
unnamed_reg_07a         0x07a[0:7]      0
unnamed_reg_07b         0x07b[0:7]      0
unnamed_reg_07c         0x07c[0:7]      0
unnamed_reg_07d         0x07d[0:7]      0
unnamed_reg_07e         0x07e[0:7]      0
unnamed_reg_07f         0x07f[0:7]      0
unnamed_reg_080         0x080[0:7]      0
unnamed_reg_081         0x081[0:7]      0
unnamed_reg_08e         0x08e[0:7]      0
unnamed_reg_08f         0x08f[0:7]      0
unnamed_reg_090         0x090[0:7]      0
unnamed_reg_091         0x091[0:7]      0
unnamed_reg_092         0x092[0:7]      0
unnamed_reg_093         0x093[0:7]      0
unnamed_reg_094         0x094[0:7]      0
unnamed_reg_095         0x095[0:7]      0
unnamed_reg_096         0x096[0:7]      0
unnamed_reg_097         0x097[0:7]      0
unnamed_reg_098         0x098[0:7]      0
unnamed_reg_099         0x099[0:7]      0
unnamed_reg_09a         0x09a[0:7]      0
unnamed_reg_09b         0x09b[0:7]      0
unnamed_reg_09c         0x09c[0:7]      0
unnamed_reg_09d         0x09d[0:7]      0
unnamed_reg_09e         0x09e[0:7]      0
unnamed_reg_09f         0x09f[0:7]      0
unnamed_reg_0a0         0x0a0[0:7]      12
unnamed_reg_0a1         0x0a1[0:7]      120
unnamed_reg_0a2         0x0a2[0:7]      31
unnamed_reg_0a3         0x0a3[0:7]      0
unnamed_reg_0a4         0x0a4[0:7]      16
unnamed_reg_0a5         0x0a5[0:7]      6
unnamed_reg_0a6         0x0a6[0:7]      6
unnamed_reg_0a7         0x0a7[0:7]      0
unnamed_reg_0a8         0x0a8[0:7]      0
unnamed_reg_0a9         0x0a9[0:7]      32
unnamed_reg_0aa         0x0aa[0:7]      10
unnamed_reg_0ab         0x0ab[0:7]      0
unnamed_reg_0ac         0x0ac[0:7]      0
unnamed_reg_0ad         0x0ad[0:7]      0
unnamed_reg_0ae         0x0ae[0:7]      24
unnamed_reg_0b0         0x0b0[0:7]      0
unnamed_reg_0b1         0x0b1[0:7]      0
unnamed_reg_0b2         0x0b2[0:7]      0
unnamed_reg_0b3         0x0b3[0:7]      0
unnamed_reg_0c0         0x0c0[0:7]      111
unnamed_reg_0c1         0x0c1[0:7]      239
unnamed_reg_0c2         0x0c2[0:7]      31
unnamed_reg_0c3         0x0c3[0:7]      31
unnamed_reg_0c4         0x0c4[0:7]      31
unnamed_reg_0c5         0x0c5[0:7]      31
unnamed_reg_0c6         0x0c6[0:7]      31
unnamed_reg_0c7         0x0c7[0:7]      42
unnamed_reg_0c8         0x0c8[0:7]      42
unnamed_reg_0c9         0x0c9[0:7]      42
unnamed_reg_0ca         0x0ca[0:7]      32
unnamed_reg_0cb         0x0cb[0:7]      0
unnamed_reg_0cc         0x0cc[0:7]      0
unnamed_reg_0d0         0x0d0[0:7]      85
unnamed_reg_0d1         0x0d1[0:7]      15
unnamed_reg_0d2         0x0d2[0:7]      31
unnamed_reg_0d3         0x0d3[0:7]      96
unnamed_reg_0d6         0x0d6[0:7]      18
unnamed_reg_0d7         0x0d7[0:7]      30
unnamed_reg_0f0         0x0f0[0:7]      0
unnamed_reg_0f1         0x0f1[0:7]      0
unnamed_reg_0f2         0x0f2[0:7]      0
unnamed_reg_0f3         0x0f3[0:7]      0
unnamed_reg_0f4         0x0f4[0:7]      0
unnamed_reg_0f5         0x0f5[0:7]      0
unnamed_reg_0f6         0x0f6[0:7]      0
unnamed_reg_0fa         0x0fa[0:7]      224
unnamed_reg_0fb         0x0fb[0:7]      8
unnamed_reg_0fc         0x0fc[0:7]      3
unnamed_reg_0fd         0x0fd[0:7]      76
unnamed_reg_0fe         0x0fe[0:7]      68
unnamed_reg_0ff         0x0ff[0:7]      0
unnamed_reg_100         0x100[0:7]      111
unnamed_reg_101         0x101[0:7]      10
unnamed_reg_102         0x102[0:7]      0
unnamed_reg_103         0x103[0:7]      8
unnamed_reg_104         0x104[0:7]      47
unnamed_reg_105         0x105[0:7]      58
unnamed_reg_106         0x106[0:7]      37
unnamed_reg_107         0x107[0:7]      63
unnamed_reg_108         0x108[0:7]      31
unnamed_reg_109         0x109[0:7]      76
unnamed_reg_10a         0x10a[0:7]      88
unnamed_reg_10b         0x10b[0:7]      0
unnamed_reg_10c         0x10c[0:7]      76
unnamed_reg_10d         0x10d[0:7]      24
unnamed_reg_10e         0x10e[0:7]      0
unnamed_reg_110         0x110[0:7]      2
unnamed_reg_111         0x111[0:7]      202
unnamed_reg_112         0x112[0:7]      74
unnamed_reg_113         0x113[0:7]      74
unnamed_reg_114         0x114[0:7]      128
unnamed_reg_115         0x115[0:7]      100
unnamed_reg_116         0x116[0:7]      101
unnamed_reg_117         0x117[0:7]      8
unnamed_reg_118         0x118[0:7]      63
unnamed_reg_119         0x119[0:7]      8
unnamed_reg_11a         0x11a[0:7]      28
unnamed_reg_11b         0x11b[0:7]      10
unnamed_reg_120         0x120[0:7]      0
unnamed_reg_121         0x121[0:7]      0
unnamed_reg_122         0x122[0:7]      0
unnamed_reg_123         0x123[0:7]      0
unnamed_reg_124         0x124[0:7]      0
unnamed_reg_125         0x125[0:7]      0
unnamed_reg_126         0x126[0:7]      0
unnamed_reg_127         0x127[0:7]      0
unnamed_reg_128         0x128[0:7]      0
unnamed_reg_129         0x129[0:7]      0
unnamed_reg_12a         0x12a[0:7]      0
unnamed_reg_12c         0x12c[0:7]      0
unnamed_reg_12d         0x12d[0:7]      0
unnamed_reg_130         0x130[0:7]      0
unnamed_reg_131         0x131[0:7]      0
unnamed_reg_132         0x132[0:7]      0
unnamed_reg_133         0x133[0:7]      0
unnamed_reg_134         0x134[0:7]      0
unnamed_reg_135         0x135[0:7]      0
unnamed_reg_136         0x136[0:7]      0
unnamed_reg_137         0x137[0:7]      8
unnamed_reg_138         0x138[0:7]      0
unnamed_reg_139         0x139[0:7]      0
unnamed_reg_13a         0x13a[0:7]      0
unnamed_reg_13b         0x13b[0:7]      0
unnamed_reg_13c         0x13c[0:7]      0
unnamed_reg_13d         0x13d[0:7]      0
unnamed_reg_13e         0x13e[0:7]      0
unnamed_reg_13f         0x13f[0:7]      0
unnamed_reg_140         0x140[0:7]      0
unnamed_reg_141         0x141[0:7]      0
unnamed_reg_142         0x142[0:7]      0
unnamed_reg_143         0x143[0:7]      0
unnamed_reg_144         0x144[0:7]      0
unnamed_reg_145         0x145[0:7]      11
unnamed_reg_146         0x146[0:7]      0
unnamed_reg_147         0x147[0:7]      16
unnamed_reg_148         0x148[0:7]      4
unnamed_reg_149         0x149[0:7]      0
unnamed_reg_14a         0x14a[0:7]      8
unnamed_reg_14b         0x14b[0:7]      0
unnamed_reg_14c         0x14c[0:7]      0
unnamed_reg_14d         0x14d[0:7]      0
unnamed_reg_14e         0x14e[0:7]      0
unnamed_reg_14f         0x14f[0:7]      0
unnamed_reg_150         0x150[0:7]      0
unnamed_reg_151         0x151[0:7]      0
unnamed_reg_152         0x152[0:7]      13
unnamed_reg_153         0x153[0:7]      34
unnamed_reg_154         0x154[0:7]      0
unnamed_reg_155         0x155[0:7]      0
unnamed_reg_156         0x156[0:7]      21
unnamed_reg_157         0x157[0:7]      177
unnamed_reg_160         0x160[0:7]      0
unnamed_reg_161         0x161[0:7]      0
unnamed_reg_162         0x162[0:7]      0
unnamed_reg_163         0x163[0:7]      0
unnamed_reg_168         0x168[0:7]      0
unnamed_reg_169         0x169[0:7]      192
unnamed_reg_16a         0x16a[0:7]      8
unnamed_reg_16b         0x16b[0:7]      8
unnamed_reg_16c         0x16c[0:7]      255
unnamed_reg_16d         0x16d[0:7]      0
unnamed_reg_16e         0x16e[0:7]      11
unnamed_reg_16f         0x16f[0:7]      24
unnamed_reg_170         0x170[0:7]      0
unnamed_reg_171         0x171[0:7]      0
unnamed_reg_172         0x172[0:7]      0
unnamed_reg_173         0x173[0:7]      0
unnamed_reg_174         0x174[0:7]      0
unnamed_reg_175         0x175[0:7]      0
unnamed_reg_176         0x176[0:7]      0
unnamed_reg_177         0x177[0:7]      0
unnamed_reg_178         0x178[0:7]      0
unnamed_reg_179         0x179[0:7]      0
unnamed_reg_17a         0x17a[0:7]      0
unnamed_reg_17b         0x17b[0:7]      0
unnamed_reg_17c         0x17c[0:7]      0
unnamed_reg_17d         0x17d[0:7]      0
unnamed_reg_17e         0x17e[0:7]      0
unnamed_reg_17f         0x17f[0:7]      0
unnamed_reg_180         0x180[0:7]      0
unnamed_reg_181         0x181[0:7]      0
unnamed_reg_182         0x182[0:7]      0
unnamed_reg_185         0x185[0:7]      16
unnamed_reg_186         0x186[0:7]      180
unnamed_reg_187         0x187[0:7]      28
unnamed_reg_188         0x188[0:7]      5
unnamed_reg_189         0x189[0:7]      48
unnamed_reg_18a         0x18a[0:7]      255
unnamed_reg_18b         0x18b[0:7]      141
unnamed_reg_18c         0x18c[0:7]      0
unnamed_reg_18d         0x18d[0:7]      100
unnamed_reg_18e         0x18e[0:7]      0
unnamed_reg_18f         0x18f[0:7]      0
unnamed_reg_190         0x190[0:7]      13
unnamed_reg_191         0x191[0:7]      6
unnamed_reg_192         0x192[0:7]      3
unnamed_reg_193         0x193[0:7]      63
unnamed_reg_194         0x194[0:7]      1
unnamed_reg_19a         0x19a[0:7]      0
unnamed_reg_19b         0x19b[0:7]      0
unnamed_reg_19c         0x19c[0:7]      0
unnamed_reg_19d         0x19d[0:7]      0
unnamed_reg_19e         0x19e[0:7]      0
unnamed_reg_19f         0x19f[0:7]      0
unnamed_reg_1a0         0x1a0[0:7]      0
unnamed_reg_1a1         0x1a1[0:7]      0
unnamed_reg_1a2         0x1a2[0:7]      0
unnamed_reg_1a3         0x1a3[0:7]      0
unnamed_reg_1a4         0x1a4[0:7]      0
unnamed_reg_1a5         0x1a5[0:7]      0
unnamed_reg_1a7         0x1a7[0:7]      0
unnamed_reg_1a8         0x1a8[0:7]      0
unnamed_reg_1a9         0x1a9[0:7]      0
unnamed_reg_1aa         0x1aa[0:7]      0
unnamed_reg_1ab         0x1ab[0:7]      0
unnamed_reg_1ac         0x1ac[0:7]      0
unnamed_reg_1ad         0x1ad[0:7]      0
unnamed_reg_1ae         0x1ae[0:7]      0
unnamed_reg_1b0         0x1b0[0:7]      0
unnamed_reg_1b1         0x1b1[0:7]      7
unnamed_reg_1b2         0x1b2[0:7]      192
unnamed_reg_1b3         0x1b3[0:7]      3
unnamed_reg_1b8         0x1b8[0:7]      8
unnamed_reg_1b9         0x1b9[0:7]      7
unnamed_reg_1ba         0x1ba[0:7]      192
unnamed_reg_1bb         0x1bb[0:7]      3
unnamed_reg_1bc         0x1bc[0:7]      8
unnamed_reg_1bd         0x1bd[0:7]      0
unnamed_reg_1be         0x1be[0:7]      0
unnamed_reg_1bf         0x1bf[0:7]      0
unnamed_reg_1c0         0x1c0[0:7]      67
unnamed_reg_1c1         0x1c1[0:7]      0
unnamed_reg_1c2         0x1c2[0:7]      0
unnamed_reg_1c3         0x1c3[0:7]      0
unnamed_reg_1c4         0x1c4[0:7]      0
unnamed_reg_1c8         0x1c8[0:7]      0
unnamed_reg_1c9         0x1c9[0:7]      0
unnamed_reg_1ca         0x1ca[0:7]      0
unnamed_reg_1cb         0x1cb[0:7]      0
unnamed_reg_1cc         0x1cc[0:7]      0
unnamed_reg_1cd         0x1cd[0:7]      0
unnamed_reg_1ce         0x1ce[0:7]      0
unnamed_reg_1cf         0x1cf[0:7]      0
unnamed_reg_1d0         0x1d0[0:7]      0
unnamed_reg_1d1         0x1d1[0:7]      0
unnamed_reg_1d2         0x1d2[0:7]      0
unnamed_reg_1d5         0x1d5[0:7]      40
unnamed_reg_1d6         0x1d6[0:7]      79
unnamed_reg_1d7         0x1d7[0:7]      19
unnamed_reg_1db         0x1db[0:7]      96
unnamed_reg_1dc         0x1dc[0:7]      3
unnamed_reg_1dd         0x1dd[0:7]      11
unnamed_reg_1de         0x1de[0:7]      3
unnamed_reg_1df         0x1df[0:7]      11
unnamed_reg_1e0         0x1e0[0:7]      3
unnamed_reg_1e1         0x1e1[0:7]      3
unnamed_reg_1e2         0x1e2[0:7]      0
unnamed_reg_1e3         0x1e3[0:7]      0
unnamed_reg_1e4         0x1e4[0:7]      1
unnamed_reg_1e5         0x1e5[0:7]      1
unnamed_reg_1e6         0x1e6[0:7]      1
unnamed_reg_1e7         0x1e7[0:7]      0
unnamed_reg_1e8         0x1e8[0:7]      96
unnamed_reg_1e9         0x1e9[0:7]      0
unnamed_reg_1ea         0x1ea[0:7]      96
unnamed_reg_1eb         0x1eb[0:7]      0
unnamed_reg_1ec         0x1ec[0:7]      96
unnamed_reg_1ed         0x1ed[0:7]      7
unnamed_reg_1ee         0x1ee[0:7]      96
unnamed_reg_1ef         0x1ef[0:7]      7
unnamed_reg_1f0         0x1f0[0:7]      204
unnamed_reg_1f1         0x1f1[0:7]      7
unnamed_reg_1f2         0x1f2[0:7]      0
unnamed_reg_1f3         0x1f3[0:7]      32
unnamed_reg_1f4         0x1f4[0:7]      0
unnamed_reg_1f5         0x1f5[0:7]      0
unnamed_reg_1f8         0x1f8[0:7]      20
unnamed_reg_1f9         0x1f9[0:7]      30
unnamed_reg_1fa         0x1fa[0:7]      1
unnamed_reg_1fb         0x1fb[0:7]      5
unnamed_reg_1fc         0x1fc[0:7]      0
unnamed_reg_201         0x201[0:7]      0
unnamed_reg_202         0x202[0:7]      0
unnamed_reg_203         0x203[0:7]      36
unnamed_reg_204         0x204[0:7]      36
unnamed_reg_205         0x205[0:7]      0
unnamed_reg_206         0x206[0:7]      0
unnamed_reg_207         0x207[0:7]      40
unnamed_reg_208         0x208[0:7]      20
unnamed_reg_209         0x209[0:7]      32
unnamed_reg_20a         0x20a[0:7]      40
unnamed_reg_20b         0x20b[0:7]      20
unnamed_reg_20c         0x20c[0:7]      40
unnamed_reg_20d         0x20d[0:7]      20
unnamed_reg_20e         0x20e[0:7]      0
unnamed_reg_20f         0x20f[0:7]      41
unnamed_reg_210         0x210[0:7]      41
unnamed_reg_211         0x211[0:7]      41
unnamed_reg_212         0x212[0:7]      39
unnamed_reg_213         0x213[0:7]      39
unnamed_reg_214         0x214[0:7]      39
unnamed_reg_215         0x215[0:7]      39
unnamed_reg_216         0x216[0:7]      39
unnamed_reg_217         0x217[0:7]      39
unnamed_reg_218         0x218[0:7]      46
unnamed_reg_219         0x219[0:7]      144
unnamed_reg_21a         0x21a[0:7]      21
unnamed_reg_21b         0x21b[0:7]      16
unnamed_reg_21c         0x21c[0:7]      144
unnamed_reg_21d         0x21d[0:7]      21
unnamed_reg_21e         0x21e[0:7]      16
unnamed_reg_21f         0x21f[0:7]      144
unnamed_reg_220         0x220[0:7]      21
unnamed_reg_221         0x221[0:7]      32
unnamed_reg_222         0x222[0:7]      32
unnamed_reg_223         0x223[0:7]      64
unnamed_reg_224         0x224[0:7]      64
unnamed_reg_225         0x225[0:7]      44
unnamed_reg_226         0x226[0:7]      0
unnamed_reg_230         0x230[0:7]      84
unnamed_reg_231         0x231[0:7]      0
unnamed_reg_232         0x232[0:7]      0
unnamed_reg_233         0x233[0:7]      0
unnamed_reg_234         0x234[0:7]      0
unnamed_reg_235         0x235[0:7]      0
unnamed_reg_236         0x236[0:7]      0
unnamed_reg_237         0x237[0:7]      0
unnamed_reg_238         0x238[0:7]      0
unnamed_reg_239         0x239[0:7]      193
unnamed_reg_23a         0x23a[0:7]      10
unnamed_reg_23b         0x23b[0:7]      128
unnamed_reg_23c         0x23c[0:7]      0
unnamed_reg_23d         0x23d[0:7]      0
unnamed_reg_23e         0x23e[0:7]      0
unnamed_reg_23f         0x23f[0:7]      0
unnamed_reg_240         0x240[0:7]      0
unnamed_reg_241         0x241[0:7]      0
unnamed_reg_242         0x242[0:7]      4
unnamed_reg_243         0x243[0:7]      13
unnamed_reg_244         0x244[0:7]      0
unnamed_reg_245         0x245[0:7]      0
unnamed_reg_246         0x246[0:7]      2
unnamed_reg_247         0x247[0:7]      0
unnamed_reg_248         0x248[0:7]      11
unnamed_reg_249         0x249[0:7]      142
unnamed_reg_24a         0x24a[0:7]      2
unnamed_reg_24b         0x24b[0:7]      23
unnamed_reg_24c         0x24c[0:7]      0
unnamed_reg_24d         0x24d[0:7]      0
unnamed_reg_24e         0x24e[0:7]      0
unnamed_reg_24f         0x24f[0:7]      0
unnamed_reg_250         0x250[0:7]      112
unnamed_reg_251         0x251[0:7]      8
###rx fastlock registers
unnamed_reg_25a         0x25a[0:7]      0
unnamed_reg_25b         0x25b[0:7]      0
unnamed_reg_25c         0x25c[0:7]      0
unnamed_reg_25d         0x25d[0:7]      0
unnamed_reg_25e         0x25e[0:7]      0
unnamed_reg_25f         0x25f[0:7]      0
###
unnamed_reg_261         0x261[0:7]      0
unnamed_reg_270         0x270[0:7]      84
unnamed_reg_271         0x271[0:7]      0
unnamed_reg_272         0x272[0:7]      0
unnamed_reg_273         0x273[0:7]      0
unnamed_reg_274         0x274[0:7]      0
unnamed_reg_275         0x275[0:7]      0
unnamed_reg_276         0x276[0:7]      0
unnamed_reg_277         0x277[0:7]      0
unnamed_reg_278         0x278[0:7]      0
unnamed_reg_279         0x279[0:7]      193
unnamed_reg_27a         0x27a[0:7]      10
unnamed_reg_27b         0x27b[0:7]      128
unnamed_reg_27c         0x27c[0:7]      0
unnamed_reg_27d         0x27d[0:7]      0
unnamed_reg_27e         0x27e[0:7]      0
unnamed_reg_27f         0x27f[0:7]      0
unnamed_reg_280         0x280[0:7]      0
unnamed_reg_281         0x281[0:7]      0
unnamed_reg_282         0x282[0:7]      4
unnamed_reg_283         0x283[0:7]      13
unnamed_reg_284         0x284[0:7]      0
unnamed_reg_285         0x285[0:7]      0
unnamed_reg_286         0x286[0:7]      2
unnamed_reg_287         0x287[0:7]      0
unnamed_reg_288         0x288[0:7]      11
unnamed_reg_289         0x289[0:7]      142
unnamed_reg_28a         0x28a[0:7]      2
unnamed_reg_28b         0x28b[0:7]      64
unnamed_reg_28c         0x28c[0:7]      0
unnamed_reg_28d         0x28d[0:7]      128
unnamed_reg_28e         0x28e[0:7]      0
unnamed_reg_28f         0x28f[0:7]      0
unnamed_reg_290         0x290[0:7]      112
unnamed_reg_291         0x291[0:7]      8
unnamed_reg_292         0x292[0:7]      0
unnamed_reg_293         0x293[0:7]      0
unnamed_reg_294         0x294[0:7]      0
unnamed_reg_295         0x295[0:7]      20
unnamed_reg_296         0x296[0:7]      0
unnamed_reg_297         0x297[0:7]      0
unnamed_reg_298         0x298[0:7]      0
unnamed_reg_299         0x299[0:7]      0
###tx fastlock registers
unnamed_reg_29a         0x29a[0:7]      0
unnamed_reg_29b         0x29b[0:7]      0
unnamed_reg_29c         0x29c[0:7]      0
unnamed_reg_29d         0x29d[0:7]      0
unnamed_reg_29e         0x29e[0:7]      0
unnamed_reg_29f         0x29f[0:7]      0
###
unnamed_reg_2a1         0x2a1[0:7]      0
unnamed_reg_2a6         0x2a6[0:7]      4
unnamed_reg_2a8         0x2a8[0:7]      0
unnamed_reg_2ab         0x2ab[0:7]      4
unnamed_reg_2ac         0x2ac[0:7]      0
unnamed_reg_2b0         0x2b0[0:7]      0
unnamed_reg_2b1         0x2b1[0:7]      0
unnamed_reg_2b2         0x2b2[0:7]      0
unnamed_reg_2b3         0x2b3[0:7]      0
unnamed_reg_2b4         0x2b4[0:7]      0
unnamed_reg_2b5         0x2b5[0:7]      0
unnamed_reg_2b6         0x2b6[0:7]      0
unnamed_reg_2b7         0x2b7[0:7]      0
unnamed_reg_2b8         0x2b8[0:7]      0
unnamed_reg_2b9         0x2b9[0:7]      0
unnamed_reg_3df         0x3df[0:7]      0
unnamed_reg_3f4         0x3f4[0:7]      0
unnamed_reg_3f5         0x3f5[0:7]      0
unnamed_reg_3f6         0x3f6[0:7]      0
unnamed_reg_3fc         0x3fc[0:7]      255
unnamed_reg_3fd         0x3fd[0:7]      255
unnamed_reg_3fe         0x3fe[0:7]      63
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
