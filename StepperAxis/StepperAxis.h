#ifndef StepperAxis_h
#define StepperAxis_h

#include "Arduino.h"
#include <JsonGenerator.h>

class StepperAxis
{
	public:
		typedef enum Direction {
			Forward, 
			Backward
		};
		typedef struct {
			size_t size;
			int *pins;
			int *values;
		} PinConfig;

		StepperAxis(int stepPulsePin, int directionPin, int enablePin, int homePin, size_t microstepSettingsSize, int *microstepSettings, PinConfig *microstepPinConfig);
		StepperAxis(int stepPulsePin, int directionPin, int enablePin, int homePin);
		StepperAxis(int stepPulsePin, int directionPin, size_t microstepSettingsSize, int *microstepSettings, PinConfig *microstepPinConfig);
		StepperAxis(int stepPulsePin, int directionPin);

		void MicroStep(int targetPulseWidth);
		void MicroStep();
		void ChangeMicroStep(int desiredMicrostep);
		void ChangeDirection(Direction dir);
		void Home(); // TODO

		void SetTargetPosition(double position);
		void UpdateMoveToTargetPosition();

		unsigned int GetPulseWidth();
		unsigned int GetNumStepsPerRev();

		ArduinoJson::Generator::JsonObject<7> GetJsonDebugInfo();
	private:
		unsigned int numStepsPerRev; // Number of steps for a full revolution
		float revLinearDistance; // The linear distance travelled from a full rotation. From Lead Screw or Gear/pulley (in mm)

		unsigned int minPulseWidth; // in microseconds
		float acceleration; // mm/second^2
		float distanceDamping; // The distance from the targetPosition to start damping the speed

		int stepPin;
		int directionPin;
		int enablePin;
		int homePin;

		double currentPositon;
		double targetPosition;

		unsigned int currentPulseWidth;
		int currentMicrostepSetting;
		float currentSpeed;

		Direction currentDirection;

		int prevPulseWidth;

		elapsedMicros sinceToggle;
		elapsedMicros deltaTimeMicroseconds;
		// Keeps track since we last sent a serial in our debug serial
		elapsedMicros sinceSerial;


		void Init(int stepPulsePin, int directionPin, int enablePin, int homePin, size_t microstepSettingsSize, int *microstepSettings, PinConfig *microstepPinConfig);

		float Clamp(float val, float minVal, float maxVal);
};

#endif