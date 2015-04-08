
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

//This function transmits the RGB command to all of LEDs.
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

int main()
{
	uint32_t LEDnum = 16; //number of LED's
	uint32_t Green = 0x00;
	uint32_t Red = 0x55;
	uint32_t Blue = 0xAA;

	setup();

	while(1){

		Green = Green+1;
		Red = Red+1;
		Blue = Blue+1;

		LED_RING_ALL(LEDnum, Red, Green, Blue);
		SysCtlDelay(100000);
	}
}

