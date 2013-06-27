#Copyright 2010 Ettus Research LLC
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
## address 0
########################################################################
address0		 0[0:4]     0          
CLKout0_1_DIV	         0[5:15]    25         div_25
CLKout0_1_HS             0[16]      0          
RESET                    0[17]      0          no_reset, reset
CLKout0_1_DDLY	         0[18:27]   0          five
CLKout0_ADLY_SEL         0[28]      0          d_pd, d_ev_x, d_odd_y, d_both
CLKout1_ADLY_SEL	 0[29]      0          d_pd, d_ev_x, d_odd_y, d_both
Required		 0[30]      0         
CLKout1_1_PD             0[31] 	    1	       power_up, power_down 
########################################################################
## address 1
########################################################################
address1		 1[0:4]     1          
CLKout0_1_DIV	         1[5:15]    25         
CLKout0_1_HS             1[16]      0          
RESET                    1[17]      0          normal, disabled
CLKout0_1_DDLY	         1[18:27]   0          
CLKout0_ADLY_SEL         1[28]      0          d_pd, d_ev_x, d_odd_y, d_both
CLKout1_ADLY_SEL	 1[29]      0          d_pd, d_ev_x, d_odd_y, d_both
Required		 1[30]      0         
CLKout1_1_PD             1[31] 	    1	       power_up, power_down
########################################################################
## address 2
########################################################################
address2		 2[0:4]     2          
CLKout0_1_DIV	         2[5:15]    25         
CLKout0_1_HS             2[16]      0          
RESET                    2[17]      0          
CLKout0_1_DDLY	         2[18:27]   0          
CLKout0_ADLY_SEL         2[28]      0          d_pd, d_ev_x, d_odd_y, d_both
CLKout1_ADLY_SEL	 2[29]      0          d_pd, d_ev_x, d_odd_y, d_both
Required		 2[30]      0         
CLKout1_1_PD             2[31] 	    1	       power_up, power_down
########################################################################
## address 3
########################################################################
address3		 3[0:4]     3          
CLKout0_1_DIV	         3[5:15]    1          
CLKout0_1_HS             3[16]      0         
RESET                    3[17]      0         
CLKout0_1_DDLY	         3[18:27]   0          
CLKout0_ADLY_SEL         3[28]      0          d_pd, d_ev_x, d_odd_y, d_both
CLKout1_ADLY_SEL	 3[29]      0          d_pd, d_ev_x, d_odd_y, d_both
CLKout6_7_OSCin_Sel      3[30]      1          VCO,OSCin
CLKout1_1_PD             3[31] 	    0	       power_up, power_down
########################################################################
## address 4
########################################################################
address4		 4[0:4]     4          
CLKout0_1_DIV	         4[5:15]    25         
CLKout0_1_HS             4[16]      0          
RESET                    4[17]      0          
CLKout0_1_DDLY	         4[18:27]   0          
CLKout0_ADLY_SEL         4[28]      0          d_pd, d_ev_x, d_odd_y, d_both
CLKout1_ADLY_SEL	 4[29]      0          d_pd, d_ev_x, d_odd_y, d_both
CLKout8_9_OSCin_Sel      4[30]      0	       VCO, OSCin         
CLKout1_1_PD             4[31] 	    0	       power_up, power_down
########################################################################
## address 5
########################################################################
address5		 5[0:4]     5          
CLKout0_1_DIV	         5[5:15]    25          
CLKout0_1_HS             5[16]      0          
RESET                    5[17]      0          
CLKout0_1_DDLY	         5[18:27]   0          
CLKout0_ADLY_SEL         5[28]      0          d_pd, d_ev_x, d_odd_y, d_both
CLKout1_ADLY_SEL	 5[29]      0          d_pd, d_ev_x, d_odd_y, d_both
Required		 5[30]      0         
CLKout1_1_PD             5[31] 	    1	       normal, power_down
########################################################################
## address 6
########################################################################
address6		 6[0:4]     6          
CLKout0_1_ADLY	         6[5:9]     0          
Required	         6[10]      0          
CLKout2_3_ADLY           6[11:15]   0         
CLKout0_TYPE	         6[16:19]   0          
CLKout1_TYPE             6[20:23]   0          
CLKout2_TYPE	         6[24:27]   0          
CLKout3_TYPE		 6[28:31]   0
########################################################################
## address 7
########################################################################
address7		 7[0:4]     7          
CLKout0_1_ADLY	         7[5:9]     0          
Required	         7[10]      0          
CLKout2_3_ADLY           7[11:15]   0         
CLKout0_TYPE	         7[16:19]   0          
CLKout1_TYPE             7[20:23]   0          
CLKout2_TYPE	         7[24:27]   0          
CLKout3_TYPE		 7[28:31]   0 
########################################################################
## address 8
########################################################################
address8		 8[0:4]     8          
CLKout0_1_ADLY	         8[5:9]     0          
Required	         8[10]      0          
CLKout2_3_ADLY           8[11:15]   0         
CLKout0_TYPE	         8[16:19]   0          
CLKout1_TYPE             8[20:23]   0          
CLKout2_TYPE	         8[24:27]   0          
CLKout3_TYPE		 8[28:31]   0 
########################################################################
## address 9
########################################################################
address9		 9[0:4]     9          
Required	         9[5]       0          
Required	         9[6]       1          
Required                 9[7]       0         
Required	         9[8]       1          
Required                 9[9]       0          
Required	         9[10]      1          
Required		 9[11]      0
Required		 9[12]      1
Required		 9[13]      0
Required		 9[14]      1
Required                 9[15]      0
Required                 9[16]      1
Required                 9[17]      0
Required                 9[18]      1          
Required                 9[19]      0          
Required                 9[20]      1         
Required                 9[21]      0          
Required                 9[22]      1          
Required                 9[23]      0          
Required                 9[24]      1
Required                 9[25]      0
Required                 9[26]      1
Required                 9[27]      0
Required                 9[28]      1
Required                 9[29]      0
Required                 9[30]      1
Required                 9[31]      0
########################################################################
## address 10
########################################################################
address10                10[0:4]    10          
FEEDBACK_MUX             10[5:7]    0          
VCO_DIV                  10[8:10]   2          
EN_FEEDBACK_MUX          10[11]     0           powered_down, enabled
VCO_MUX                  10[12]     0           just_vco, vco_divider
Required                 10[13]     0          
Required                 10[14]     1          
Required                 10[15]     0
OSCout_DIV               10[16:18]  0		 
PD_OSCin                 10[19]     0		 normal, power_down
OSCout10_MUX             10[20]     0		 bypass_div, divided
Required                 10[21]     0		 
EN_OSCout0               10[22]     1		 disabled, enabled
Required                 10[23]     0
OSCout0_TYPE             10[24:27]  1
Required                 10[28]     1
Required                 10[29:31]  0
########################################################################
## address 11
########################################################################
address11		 11[0:4]    11
EN_PLL2_XTAL 		 11[5]      0
Required		 11[6:11]   0
SYNC_TYPE                11[12:14]  0
SYNC_EN_AUTO             11[15]     0
SYNC_POL_INV             11[16]     0
SYNC_QUAL                11[17]     0
SYNC_CLKin2_MUX          11[18:19]  0
NO_SYNC_CLKout0_1        11[20]     0
NO_SYNC_CLKout2_3        11[21]     0
NO_SYNC_CLKout4_5        11[22]     0
NO_SYNC_CLKout6_7        11[23]     0
NO_SYNC_CLKout8_9        11[24]     0
NO_SYNC_CLKout10_11      11[25]     0
EN_SYNC                  11[26]     0
MODE                     11[27:31]  0
########################################################################
## address 12
########################################################################
address12		 12[0:4]    0
HOLDOVER_MODE		 12[6:7]    0
Required                 12[9:17]   0
Required	         12[18:19]  1
Required                 12[20]     0
Required_LE              12[21]     0
SYNC_PLL1_DLD            12[22]     0
SYNC_PLL2_DLD            12[23]     0
LD_TYPE                  12[24:26]  0
LD_MUX                   12[27:31]  0
########################################################################
## address 13
########################################################################
address13		 13[0:4]    0
EN_CLKin0		 13[5]      0
EN_CLKin1                13[6]      0
EN_CLKin2                13[7]      0
CLKin_Sel_INV            13[8]      0
CLKin_Select_MODE        13[8:11]   0
Status_CLKin0_MUX        13[14:12]  0
DISABLE_DLD1_DET         13[13]     0
Status_CLKin0_TYPE       13[16:18]  0
Required                 13[19]     0
Status_CLKin1_MUX        13[20:22]  0
Required                 13[23]     0
HOLDOVER_TYPE            13[24:26]  0
HOLDOVER_MUX             13[27:31]  0
########################################################################
## address 14
########################################################################
address14		 14[0:4]    14
EN_VTUNE_RAIL_DET	 14[5]      0
DAC_LOW_TRIP             14[6:11]   0
Required                 14[12:13]  0
DAC_HIGH_TRIP            14[14:19]  0
CLKin0_BUF_TYPE          14[20]     0
CLKin1_BUF_TYPE          14[21]     0
CLKin2_BUF_TYPE          14[22]     0
Required                 14[23]     0
Status_CLKin1_TYPE       14[24:26]  0
Required                 14[27]     0
EN_LOS                   14[28]     0
Required                 14[29]     0
LOS_TIMEOUT              14[30:31]  0
########################################################################
## address 15
########################################################################
address15 		 15[0:4]    15
FORCE_HOLDOVER           15[5]      0
HOLDOVER_DLD_CNT         15[6:19]   0
EN_MAN_DAC               15[20]     0
Required                 15[21]     0
MAN_DAC                  15[22:31]  0

########################################################################
## address 16
########################################################################
address16                16[0:4]     0
Required                 16[5]       0          
Required                 16[6]       0          
Required                 16[7]       0         
Required                 16[8]       0          
Required                 16[9]       0          
Required                 16[10]      1          
Required                 16[11]      0
Required                 16[12]      0
Required                 16[13]      0
Required                 16[14]      0
Required                 16[15]      0
Required                 16[16]      1
Required                 16[17]      0
Required                 16[18]      1          
Required                 16[19]      0          
Required                 16[20]      1         
Required                 16[21]      0          
Required                 16[22]      1          
Required                 16[23]      0  
Required                 16[24]      1
Required                 16[25]      0
Required                 16[26]      0
Required                 16[27]      0
Required                 16[28]      0
Required                 16[29]      0
XTAL_LVL		 16[30:31]   0
########################################################################
## address 24
########################################################################
address24		 24[0:4]     24
PLL1_WND_SIZE		 24[6:7]     0
PLL1_R_DLY               24[8:10]    0
Required                 24[11]      0
PLL2_N_DLY		 24[12:14]   0
Required                 24[15]      0
PLL2_R3_LF               24[16:18]   0
Required                 24[19]      0
PLL2_R4_LF               24[20:22]   0
Required                 24[23]      0
PLL2_C3_LF               24[24:27]   0
PLL2_C4_LF               24[28:31]   0
#########################################################################
## address 25
#########################################################################
addres25		 25[0:4]     25
PLL2_CP_TRI		 25[5]       0
PLL1_DLD_CNT             25[6:19]    0
Required                 25[20:21]   0
DAC_CLK_DIV		 25[22:31]   0
#########################################################################
## address 26
#########################################################################
address26		 26[0:4]     26
PLL2_CP_TRI              26[5]       0
PLL2_DLD_CNT		 26[6:19]    0
Required                 26[20]      0
Required                 26[21]      1
Required                 26[22]      0
Required                 26[23]      1
Required                 26[24]      1
Required                 26[25]      1
PLL2_CP_GAIN             26[26:27]   0
PLL2_CP_POL              26[28]      0
EN_PLL2_REF_2X           26[29]      0
PLL2_WND_SIZE            26[30:31]
#########################################################################
## address 27
#########################################################################
address27		 27[0:4]     27
PLL1_CP_TRI              27[5]       0
PLL1_R                   27[6:19]    0
CLKin0_PreR_DIV		 27[20:21]   0
CLKin1_PreR_DIV          27[22:23]   0
CLKin2_PreR_DIV          27[24:25]   0
PLL1_CP_GAIN             27[26:27]   0
PLL1_CP_POL              27[28]      0
Reqiured                 27[29:31]   0
#########################################################################
## address 28
#########################################################################
address28                28[0:4]     28
Required                 28[5]       0
PLL1_N                   28[6:19]    0
PLL2_R                   28[20:31]   0
#########################################################################
## address 29
#########################################################################
address29                29[0:4]     29
PLL2_N_CAL               29[5:22]    0
PLL2_FAST_PDF            29[23]      0
OSCin_FREQ               29[24:26]   0
Required                 29[27:31]   0
#########################################################################
## address 30
#########################################################################
address30		 30[0:4]     30
PLL2_N			 30[5:22]    0
Required                 30[23]      0
PLL2_P                   30[24:26]   0
Required                 30[27:31]   0
#########################################################################
## address 31
#########################################################################
address31                31[0:4]     0
uWire_LOCK               31[5]       0
Required                 31[6:15]    0
READBACk_ADDR            31[16:20]   0
READBACK_LE              31[21]      0
Required                 31[22:31]   0 








            
