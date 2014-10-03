# Stepper-Axis

Stepper Motor library for the Teensy/Arduino.

This project is not recommended for public use. 
 - The API is still under development
 - Does not behave correctly at the moment
 
# Dependencies:
 - [ArduinoJSON](https://github.com/bblanchon/ArduinoJson): For debug information and target reach callbacks


# Usage:

```
int stepPin = 0;
int dirPin = 1;
StepperAxis xAxis(stepPin, dirPin);
```
