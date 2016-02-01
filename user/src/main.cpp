// Includes --------------------------------------------------------------------
#include "main.h"

// Private variables -----------------------------------------------------------
Ioton ton_fit;
Ticker flipper_1;
PwmOut led_ex(PIN13);
AnalogIn potentiometer(PIN15);
float potentiometerValue = 0.0;

// Flipper 0.1 s - 10 Hz -------------------------------------------------------
void flip_1( void )
{
	// Read potentiometer
	potentiometerValue = potentiometer;

	// Update duty cycle
	led_ex = potentiometerValue;
}

// *****************************************************************************
// MAIN PROGRAM ****************************************************************
int main( void )
{
	// Configure function to be attached (flip) and the interval (0.1 second)
	flipper_1.attach(&flip_1, 0.1);

	// The main LOOP ***********************************************************
	while (1)
	{
		// Add your repeated code here
		ton_fit.setLED(RED);
		wait(potentiometerValue);
		ton_fit.setLED(GREEN);
		wait(potentiometerValue);
		ton_fit.setLED(BLUE);
		wait(potentiometerValue);
		ton_fit.setLED(YELLOW);
		wait(potentiometerValue);
		ton_fit.setLED(CYAN);
		wait(potentiometerValue);
		ton_fit.setLED(MAGENTA);
		wait(potentiometerValue);
		ton_fit.setLED(WHITE);
		wait(potentiometerValue);
		ton_fit.setLED(NONE);
		wait(potentiometerValue);
	} // end of main LOOP

} // end of main function
