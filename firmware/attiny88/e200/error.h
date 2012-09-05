/*
 * error.h
 *
 * Created: 4/09/2012 6:25:53 PM
 *  Author: Balint Seeber
 */ 


#ifndef ERROR_H_
#define ERROR_H_

enum ErrorBlinkCount	// Lower number = higher priority
{
	BlinkError_None,
	// Low power/battery
	BlinkError_LTC3675_UnderVoltage,
	// Should match power boot steps
	BlinkError_FPGA_Power,
	BlinkError_DRAM_Power,
	BlinkError_1_8V_Peripherals_Power,
	BlinkError_3_3V_Peripherals_Power,
	BlinkError_TX_Power,
	// LTC3675
	BlinkError_LTC3675_OverTemperature,
	// LTC4155
};

#endif /* ERROR_H_ */
