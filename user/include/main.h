#ifndef MAIN_H
#define MAIN_H

#include "mbed.h"

#define ON	1
#define OFF	0

typedef enum
{
	RED,
	GREEN,
	BLUE,
	YELLOW,
	CYAN,
	MAGENTA,
	WHITE,
	NONE
} LEDType_t;


class Ioton
{
public:
	void setLED(PinName led, int state, float intensity = 1.0f)
	{
		if (state == OFF || intensity == 1.0f)
		{
			DigitalOut pin(led);
			pin = state;
		}
		else
		{
			PwmOut pin(led);

			if (intensity > 1.0f)
			{
				intensity = 1.0f;
			}
			else if (intensity < 0.0f)
			{
				intensity = 0.0f;
			}

			pin = intensity;
		}
	}

	void setLED(LEDType_t color)
	{
		setLED(LED_RED, OFF);
		setLED(LED_GREEN, OFF);
		setLED(LED_BLUE, OFF);

		switch(color)
		{
			case RED:
				setLED(LED_RED, ON);
				break;

			case GREEN:
				setLED(LED_GREEN, ON);
				break;

			case BLUE:
				setLED(LED_BLUE, ON);
				break;

			case YELLOW:
				setLED(LED_RED, ON);
				setLED(LED_GREEN, ON);
				break;

			case CYAN:
				setLED(LED_GREEN, ON);
				setLED(LED_BLUE, ON);
				break;

			case MAGENTA:
				setLED(LED_RED, ON);
				setLED(LED_BLUE, ON);
				break;

			case WHITE:
				setLED(LED_RED, ON);
				setLED(LED_GREEN, ON);
				setLED(LED_BLUE, ON);
				break;

			case NONE:
				break;

			default:
				break;
		}
	}
};

#endif // MAIN_H
