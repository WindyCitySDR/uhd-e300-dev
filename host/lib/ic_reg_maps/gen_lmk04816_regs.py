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
CLKout0_ADLY_SEL         0[28]      0          
CLKout1_ADLY_SEL	 0[29]      0          
Required		 0[30]      0         
CLKout1_1_PD             0[31] 	    1	       normal, power_down 
########################################################################
## address 1
########################################################################
address1		 1[0:4]     1          
CLKout0_1_DIV	         1[5:15]    25         
CLKout0_1_HS             1[16]      0          
RESET                    1[17]      0          normal, disabled
CLKout0_1_DDLY	         1[18:27]   0          
CLKout0_ADLY_SEL         1[28]      0          
CLKout1_ADLY_SEL	 1[29]      0          
Required		 1[30]      0         
CLKout1_1_PD             1[31] 	    1	       normal, power_down
########################################################################
## address 2
########################################################################
address2		 2[0:4]     2          
CLKout0_1_DIV	         2[5:15]    25         
CLKout0_1_HS             2[16]      0          
RESET                    2[17]      0          
CLKout0_1_DDLY	         2[18:27]   0          
CLKout0_ADLY_SEL         2[28]      0          
CLKout1_ADLY_SEL	 2[29]      0          
Required		 2[30]      0         
CLKout1_1_PD             2[31] 	    1	       normal, power_down
########################################################################
## address 3
########################################################################
address3		 3[0:4]     3          
CLKout0_1_DIV	         3[5:15]    1          
CLKout0_1_HS             3[16]      0         
RESET                    3[17]      0         
CLKout0_1_DDLY	         3[18:27]   0          
CLKout0_ADLY_SEL         3[28]      0          
CLKout1_ADLY_SEL	 3[29]      0          
Required		 3[30]      1          VCO,OSCin
CLKout1_1_PD             3[31] 	    0	       normal, power_down
########################################################################
## address 4
########################################################################
address4		 4[0:4]     4          
CLKout0_1_DIV	         4[5:15]    25         
CLKout0_1_HS             4[16]      0          
RESET                    4[17]      0          
CLKout0_1_DDLY	         4[18:27]   0          
CLKout0_ADLY_SEL         4[28]      0          
CLKout1_ADLY_SEL	 4[29]      0          
Required		 4[30]      0	       VCO, OSCin         
CLKout1_1_PD             4[31] 	    0	       normal, power_down
########################################################################
## address 5
########################################################################
address5		 5[0:4]     5          
CLKout0_1_DIV	         5[5:15]    0          
CLKout0_1_HS             5[16]      0          3state, dld, ndiv, dvdd, rdiv, nchan_od_ld, sdo, dgnd
RESET                    5[17]      0          neg, pos
CLKout0_1_DDLY	         5[18:27]   0          normal, 3state
CLKout0_ADLY_SEL         5[28]      0          set1, set2
CLKout1_ADLY_SEL	 5[29]      0          
Required		 5[30]      0         
CLKout1_1_PD             5[31] 	    1	       normal, power_down
########################################################################
## address 6
########################################################################
address1		 0[0:4]     6          
CLKout0_1_DIV	         0[5:15]    0          normal, reset
CLKout0_1_HS             0[16]      0          3state, dld, ndiv, dvdd, rdiv, nchan_od_ld, sdo, dgnd
RESET                    0[17]      0          neg, pos
CLKout0_1_DDLY	         0[18:27]   0          normal, 3state
CLKout0_ADLY_SEL         0[28]      0          set1, set2
CLKout1_ADLY_SEL	 0[29]      0          
Required		 0[30]      0         
CLKout1_1_PD             0[31] 	    1	       power_up,power_down
