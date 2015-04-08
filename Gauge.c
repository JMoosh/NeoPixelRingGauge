
//*****************************************************************************
//
// WS2812 example with Stellaris Launchpad (EK-LM4F120XL)
// By: Jordan Mosher, "Improvements" on the code of Zach Archer
//
// Requirement: http://www.adafruit.com/datasheets/WS2812.pdf
//
// Setup: Connect the WS2812 data line to pin PA_5.
//

#include "stdint.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"

void setup()
{
	SysCtlPeripheralEnable( SYSCTL_PERIPH_SSI0 );
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	SSIDisable( SSI0_BASE );
	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	GPIOPinConfigure(GPIO_PA5_SSI0TX);
	GPIOPinTypeSSI(GPIO_PORTA_BASE,GPIO_PIN_5|GPIO_PIN_3|GPIO_PIN_2);

	/*
	  Polarity Phase        Mode
	     0 	   0   SSI_FRF_MOTO_MODE_0
	     0     1   SSI_FRF_MOTO_MODE_1
	     1     0   SSI_FRF_MOTO_MODE_2
	     1     1   SSI_FRF_MOTO_MODE_3
	*/

	//SSIClockSourceSet( SSI0_BASE, SSI_CLOCK_SYSTEM );

        // The actual desired bit rate, 2400000, produces weird color artifacts. :P 2150000 seems OK.
        // SSI_FRF_MOTO_MODE_0 breaks the timing when the first LED is lit. (Zach Archer)

	SSIConfigSetExpClk( SSI0_BASE,
                                SysCtlClockGet(),
                                SSI_FRF_MOTO_MODE_1,
	                        SSI_MODE_MASTER,
                                2150000,
                                8 );
	SSIEnable( SSI0_BASE );
}

//This function converts a single color byte into a 3byte baud byte packed into a uint32_t

uint32_t
Byte2Baud(uint8_t input)
{
	int8_t rgbBitIdx;
	uint32_t BaseBaud = 9586980; //100100100100100100100100
	for( rgbBitIdx=7; rgbBitIdx>=0; rgbBitIdx-- ) {
		if (input & (0x1<<rgbBitIdx)){
			BaseBaud = BaseBaud | 0x1<<((rgbBitIdx*3)+1);
		}
	}
	return BaseBaud;
}

//This function transmits the RGP command to all of LEDs.
// 3-baud instructions for WS2812:
//   0: ^__  (low)
//   1: ^^_  (HIGH)

void
LED_RING_ALL(uint32_t LEDnum, uint32_t Red, uint32_t Green, uint32_t Blue)
{
	int8_t LEDIdx;
	uint8_t LEDtxIdx;
	int8_t buffer[144]; // = (LEDnum*9)
	uint32_t RedBaud = Byte2Baud(Red);
	uint32_t GreenBaud = Byte2Baud(Green);
	uint32_t BlueBaud = Byte2Baud(Blue);


		for( LEDIdx=0; LEDIdx < LEDnum; LEDIdx++ ) {
			buffer[(LEDIdx*9)]     = (GreenBaud>>16);
			buffer[((LEDIdx*9)+1)] = (GreenBaud>>8);
			buffer[((LEDIdx*9)+2)] = (GreenBaud);
			buffer[((LEDIdx*9)+3)] = (RedBaud>>16);
			buffer[((LEDIdx*9)+4)] = (RedBaud>>8);
			buffer[((LEDIdx*9)+5)] = (RedBaud);
			buffer[((LEDIdx*9)+6)] = (BlueBaud>>16);
			buffer[((LEDIdx*9)+7)] = (BlueBaud>>8);
			buffer[((LEDIdx*9)+8)] = (BlueBaud);
		}

		// Wait until SSI0_BASE is ready to send bytes
		while(SSIBusy( SSI0_BASE ));

		for(LEDtxIdx = 0; LEDtxIdx < (LEDnum*9); LEDtxIdx++) {
			uint8_t data = buffer[LEDtxIdx];
			SSIDataPut( SSI0_BASE, data);
		}
}

void
LED_RING_Percent(uint32_t LEDnum, uint8_t Start, uint8_t End, uint32_t Red, uint32_t Green, uint32_t Blue, uint32_t BG_Red, uint32_t BG_Green, uint32_t BG_Blue)
{
	int8_t LEDIdx;
	uint8_t LEDtxIdx;
	int8_t buffer[144]; // = (LEDnum*9)

	// Create Color Bauds for Display Color
	uint32_t RedBaud = Byte2Baud(Red);
	uint32_t GreenBaud = Byte2Baud(Green);
	uint32_t BlueBaud = Byte2Baud(Blue);

	// Create Color Bauds for the Background Color
	uint32_t BG_RedBaud = Byte2Baud(BG_Red);
	uint32_t BG_GreenBaud = Byte2Baud(BG_Green);
	uint32_t BG_BlueBaud = Byte2Baud(BG_Blue);

	// Select Start and End LEDs
	uint8_t StartLED = Start/(256/LEDnum);
	uint8_t EndLED = End/(256/LEDnum);

	// Write the first to Start Color
	for( LEDIdx=0; LEDIdx < (StartLED-1); LEDIdx++ ) {
		buffer[(LEDIdx*9)]     = (BG_GreenBaud>>16);
		buffer[((LEDIdx*9)+1)] = (BG_GreenBaud>>8);
		buffer[((LEDIdx*9)+2)] = (BG_GreenBaud);
		buffer[((LEDIdx*9)+3)] = (BG_RedBaud>>16);
		buffer[((LEDIdx*9)+4)] = (BG_RedBaud>>8);
		buffer[((LEDIdx*9)+5)] = (BG_RedBaud);
		buffer[((LEDIdx*9)+6)] = (BG_BlueBaud>>16);
		buffer[((LEDIdx*9)+7)] = (BG_BlueBaud>>8);
		buffer[((LEDIdx*9)+8)] = (BG_BlueBaud);
	}
	// Write the Start To End Color
	for( LEDIdx=StartLED; LEDIdx < (EndLED+1); LEDIdx++ ) {
		buffer[(LEDIdx*9)]     = (GreenBaud>>16);
		buffer[((LEDIdx*9)+1)] = (GreenBaud>>8);
		buffer[((LEDIdx*9)+2)] = (GreenBaud);
		buffer[((LEDIdx*9)+3)] = (RedBaud>>16);
		buffer[((LEDIdx*9)+4)] = (RedBaud>>8);
		buffer[((LEDIdx*9)+5)] = (RedBaud);
		buffer[((LEDIdx*9)+6)] = (BlueBaud>>16);
		buffer[((LEDIdx*9)+7)] = (BlueBaud>>8);
		buffer[((LEDIdx*9)+8)] = (BlueBaud);
	}
	// Write the End To Last Color
	for( LEDIdx=(EndLED+1); LEDIdx < (LEDnum+1); LEDIdx++ ) {
		buffer[(LEDIdx*9)]     = (BG_GreenBaud>>16);
		buffer[((LEDIdx*9)+1)] = (BG_GreenBaud>>8);
		buffer[((LEDIdx*9)+2)] = (BG_GreenBaud);
		buffer[((LEDIdx*9)+3)] = (BG_RedBaud>>16);
		buffer[((LEDIdx*9)+4)] = (BG_RedBaud>>8);
		buffer[((LEDIdx*9)+5)] = (BG_RedBaud);
		buffer[((LEDIdx*9)+6)] = (BG_BlueBaud>>16);
		buffer[((LEDIdx*9)+7)] = (BG_BlueBaud>>8);
		buffer[((LEDIdx*9)+8)] = (BG_BlueBaud);
	}


	// Wait until SSI0_BASE is ready to send bytes
	while(SSIBusy( SSI0_BASE ));

	for(LEDtxIdx = 0; LEDtxIdx < (LEDnum*9); LEDtxIdx++) {
		uint8_t data = buffer[LEDtxIdx];
		SSIDataPut( SSI0_BASE, data);
	}
}

int main()
{
	uint8_t Start = 1;
	uint8_t End = 128;
	uint32_t LEDnum = 16; //number of LED's

	// Foreground Color
	uint32_t Green = 0x00;
	uint32_t Red = 0x00;
	uint32_t Blue = 0x20;

	// Background Color
	uint32_t BG_Green = 0x00;
	uint32_t BG_Red = 0x08;
	uint32_t BG_Blue = 0x00;

	setup();

	while(1){

		//Color changing to detect updates
		//Green = Green+1;
		//Red = Red+1;
		//Blue = Blue+1;
		End = End+1;

		LED_RING_Percent(LEDnum, Start, End, Red, Green, Blue, BG_Red, BG_Green, BG_Blue);
		SysCtlDelay(100000);
	}
}

