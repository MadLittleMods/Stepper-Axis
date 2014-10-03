# Stepper-Axis

Stepper Motor library for the Teensy/Arduino.

This project is not recommended for public use. 
 - The API is still under development
 - Does not behave correctly at the moment
 
# Dependencies:
 - [ArduinoJSON](https://github.com/bblanchon/ArduinoJson): For debug information and target reach callbacks


# Usage:

```
#include <StepperAxis.h>
#include <JsonParser.h> // https://github.com/bblanchon/ArduinoJson

int stepPin = 0;
int dirPin = 1;
 
void setup()
{	
	StepperAxis xAxis(stepPin, dirPin);
}
```
