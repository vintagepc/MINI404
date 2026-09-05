// fan.h - Include file for fan.c with enums for the pwm input...

enum FAN_PWM_INPUT
{
	FAN_PWM_INPUT = 0,
	FAN_PWM_INPUT_INVERTED,
	FAN_PWM_INPUT_COUNT
};

// Cooling-fan indexing, shared between the machine setup and heater's fan model inputs
enum FAN_COOLING
{
	FAN_COOLING_PRINT = 0,
	FAN_COOLING_HBR,
	FAN_COOLING_COUNT
};
