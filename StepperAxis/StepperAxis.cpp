#include "Arduino.h"
#include "StepperAxis.h"

#include <JsonGenerator.h>
using namespace ArduinoJson::Generator;





StepperAxis::StepperAxis(int stepPulsePin, int directionPin, int enablePin, int homePin, size_t microstepSettingsSize, int *microstepSettings, PinConfig *microstepPinConfig)
{
	this->Init(stepPulsePin, directionPin, enablePin, homePin, microstepSettingsSize, microstepSettings, microstepPinConfig);
}

StepperAxis::StepperAxis(int stepPulsePin, int directionPin, int enablePin, int homePin)
{
	this->Init(stepPulsePin, directionPin, enablePin, homePin, 0, NULL, NULL);
}


StepperAxis::StepperAxis(int stepPulsePin, int directionPin, size_t microstepSettingsSize, int *microstepSettings, PinConfig *microstepPinConfig)
{
	this->Init(stepPulsePin, directionPin, -1, -1, microstepSettingsSize, microstepSettings, microstepPinConfig);
}

StepperAxis::StepperAxis(int stepPulsePin, int directionPin)
{
	this->Init(stepPulsePin, directionPin, -1, -1, 0, NULL, NULL);
}


void StepperAxis::Init(int stepPulsePin, int directionPin, int enablePin, int homePin, size_t microstepSettingsSize, int *microstepSettings, PinConfig *microstepPinConfig)
{
	this->numStepsPerRev = 200; // Number of steps for a full revolution
	this->revLinearDistance = 1.27; // (in mm) The linear distance travelled from a full rotation. From Lead Screw or Gear/pulley 

	this->minPulseWidth = 150; // in microseconds
	this->acceleration = 1; // mm/second^2
	this->distanceDamping = 5; // The distance from the targetPosition to start damping the speed

	// Default to negative, we will set to what is passed in below
	this->stepPin = -1;
	this->directionPin = -1;
	this->enablePin = -1;
	this->homePin = -1;

	//this->microstepSettings[];
	//this->microstepPinConfig[];

	this->currentPositon = 0; // In mm, what position is this axis at
	this->targetPosition = 0;

	this->currentPulseWidth = 50; // In microseconds
	this->currentMicrostepSetting = 1;
	this->currentSpeed = 0; // mm/second

	this->currentDirection = Direction::Forward;

	this->prevPulseWidth = 50;



	if(stepPulsePin >= 0)
	{
		this->stepPin = stepPulsePin;
		pinMode(this->stepPin, OUTPUT);
		digitalWrite(this->stepPin, LOW); // Init
	}
	if(directionPin >= 0)
	{
		this->directionPin = directionPin;
		pinMode(this->directionPin, OUTPUT);
		digitalWrite(this->directionPin, HIGH); // Init
	}
	if(enablePin >= 0) 
	{
		this->enablePin = enablePin;
		pinMode(this->enablePin, OUTPUT);
		digitalWrite(this->enablePin, HIGH); // Init
	}
	if(homePin >= 0)
	{
		this->homePin = homePin;
		pinMode(this->homePin, INPUT);
	}

	//this->microstepSettings = microstepSettings;
	//this->microstepPinConfig = microstepPinConfig;
	//for(int i = 0; i < microstepSettingsSize; i++)
	//{

	//}


	/*
	// Loop through the the first microstep setting and initialie any pins that are used
	// The pins should be shared among the settings so we only have to do the first one
	for(int k = 0; k < this->microstepPinConfig[0].pins.size(); k++)
	{
		pinMode(this->microstepPinConfig[0].pins[k], OUTPUT);
		digitalWrite(this->microstepPinConfig[0].pins[k], HIGH); // Init
	}
	*/
}


void StepperAxis::MicroStep(int targetPulseWidth)
{
	int pulseWidth = 0;
	if(this->prevPulseWidth < targetPulseWidth)
		pulseWidth = this->Clamp((this->prevPulseWidth+10), this->prevPulseWidth, targetPulseWidth);
	else
		pulseWidth = this->Clamp((this->prevPulseWidth-10), targetPulseWidth, this->prevPulseWidth);


	/* * /
	String output = "";
	output += String(prevPulseWidth) + " : " + String(pulseWidth) + " : " + String(targetPulseWidth) + "\n";
	Serial.print(output);
	/* */

	// Make the rising edge
	digitalWrite(this->stepPin, HIGH);
	delayMicroseconds(pulseWidth);
	// Make the falling edge
	digitalWrite(this->stepPin, LOW);
	delayMicroseconds(pulseWidth);

	this->prevPulseWidth = pulseWidth;

	// Update the current position
	this->currentPositon += (this->currentDirection == Direction::Forward ? (float)1 : (float)-1) * (this->revLinearDistance/(double)(this->numStepsPerRev * this->currentMicrostepSetting));
}
void StepperAxis::MicroStep()
{
	this->MicroStep(50);
}

void StepperAxis::ChangeMicroStep(int desiredMicrostep)
{
	// TODO: Contrain to microstep config
	this->currentMicrostepSetting = desiredMicrostep;
}

void StepperAxis::ChangeDirection(Direction dir)
{
	if(dir == Direction::Forward)
	{
		digitalWrite(this->directionPin, HIGH);
	}
	else if(dir == Direction::Backward)
	{
		digitalWrite(this->directionPin, LOW);
	}

	this->currentDirection = dir;
}


void StepperAxis::Home()
{
	if(this->homePin > 0)
	{
		// TODO
	}
}

void StepperAxis::SetTargetPosition(double position)
{
	this->targetPosition = position;

}

void StepperAxis::UpdateMoveToTargetPosition()
{
	double deltaTime = (double)this->deltaTimeMicroseconds/1000000;



	if(this->targetPosition > this->currentPositon)
		this->ChangeDirection(Direction::Forward);
	else
		this->ChangeDirection(Direction::Backward);

	double positionDelta = this->targetPosition - this->currentPositon;

	float scalar = this->Clamp(abs(positionDelta), 0, this->distanceDamping)/this->distanceDamping;
	double maxStepRate = (double)1/(2 * ((double)this->minPulseWidth/1000000));
	double maxSpeed = (maxStepRate / (this->numStepsPerRev * this->currentMicrostepSetting)) * this->revLinearDistance;
	this->currentSpeed += this->Clamp(scalar * this->acceleration * deltaTime, 0, maxSpeed);

	//Serial.print("Max Speed: " + String(maxSpeed) + "\nMaxStepRate: " + String(maxStepRate) + "\nnumStepsPerRev: " + String(this->numStepsPerRev) + "\ncurrentMicrostepSetting: " + String(this->currentMicrostepSetting) + "\n");

	float stepRate = (this->numStepsPerRev * this->currentSpeed * this->currentMicrostepSetting)/this->revLinearDistance;

	this->currentPulseWidth = ((double)1/(2*stepRate))*1000000;

	// Sanitize the pulse width
	unsigned int pulseWidth = this->Clamp(this->currentPulseWidth, this->minPulseWidth, this->currentPulseWidth);

	//pulseWidth = 50;

	if(abs(positionDelta) > 0.0001)
	{
		// Toggle the pulse pin when necessary
		if(this->sinceToggle > pulseWidth)
		{
			// Toggle the pulse pin
			digitalWrite(this->stepPin, !digitalRead(this->stepPin));

			// Update the current position
			// We only step on low pulse
			if(digitalRead(this->stepPin) == LOW)
			{
				this->currentPositon += (this->currentDirection == Direction::Forward ? (float)1 : (float)-1) * (this->revLinearDistance/(this->numStepsPerRev*this->currentMicrostepSetting));
			}

			this->sinceToggle = 0;
		}
	}

	/* */
	if (this->sinceSerial >= .5 * 1000000)
	{
		//Serial.print("Speed: " + String(this->currentSpeed) + "\npos: " + String(this->currentPositon) + "\ncurPulseWidth: " + String(this->currentPulseWidth) + "\ndeltaTime: " + String(this->deltaTimeMicroseconds) + "\n");
		//Serial.print(String(pulseWidth) + "|");
		
		this->sinceSerial = 0;
	}
	/* */


	this->deltaTimeMicroseconds = 0;
}

JsonObject<7> StepperAxis::GetJsonDebugInfo()
{
	String pinString = "";
	pinString.concat("s").concat(String(this->stepPin)).concat(",d").concat(String(this->directionPin));

	JsonObject<7> jsonDebugObject;
	jsonDebugObject["type"] = "debug";
	jsonDebugObject["current_speed"] = this->currentSpeed;
	jsonDebugObject["current_position"] = this->currentPositon;
	jsonDebugObject["current_direction"] = this->currentDirection == Direction::Forward ? "Forward" : "Backward";
	jsonDebugObject["current_pulse_width"] = this->currentPulseWidth;

	jsonDebugObject["target_position"] = this->targetPosition;
	jsonDebugObject["pins"] = pinString;

	return jsonDebugObject;
}

unsigned int StepperAxis::GetPulseWidth()
{
	return this->currentPulseWidth;
}

unsigned int StepperAxis::GetNumStepsPerRev()
{
	return this->numStepsPerRev;
}




float StepperAxis::Clamp(float val, float minVal, float maxVal)
{
	float tempMax = max(val, minVal);
	return min(tempMax, maxVal);
}