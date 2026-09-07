// Three-microphone sound detector and 2D localizer for Teensy 4.0.
// Mic 0 is the origin, mic 1 is on the positive X axis, and mic 2 is
// above the X axis. All microphones must share ground and be biased near
// half the ADC supply voltage.

#include <Arduino.h>
#include <math.h>

const uint8_t MIC_PINS[3] = {A2, A3, A4};

const float MIC_SPACING_METERS = 0.10f;
const float SPEED_OF_SOUND_METERS_PER_SECOND = 343.0f;
uint16_t soundThreshold = 180;
const uint32_t SAMPLE_INTERVAL_US = 100;
const uint32_t MAX_EVENT_LENGTH_US = 100000;
const uint32_t EVENT_COOLDOWN_MS = 200;

uint16_t microphoneBias[3] = {2048, 2048, 2048};
uint32_t nextSampleTime;
uint32_t lastEventTime;
bool eventActive = false;
uint32_t eventStartTime;
uint32_t firstArrivalTime[3];
bool arrivalFound[3];
uint16_t previousLevel[3] = {0, 0, 0};

float microphoneX[3];
float microphoneY[3];

void calibrateMicrophones() {
	const uint16_t sampleCount = 500;
	uint32_t totals[3] = {0, 0, 0};

	Serial.println("Calibrating microphones. Keep the area quiet...");
	for (uint16_t sample = 0; sample < sampleCount; sample++) {
		for (uint8_t mic = 0; mic < 3; mic++) {
			totals[mic] += analogRead(MIC_PINS[mic]);
		}
		delayMicroseconds(1000);
	}

	for (uint8_t mic = 0; mic < 3; mic++) {
		microphoneBias[mic] = totals[mic] / sampleCount;
	}
}

void setupMicrophoneGeometry() {
	microphoneX[0] = 0.0f;
	microphoneY[0] = 0.0f;
	microphoneX[1] = MIC_SPACING_METERS;
	microphoneY[1] = 0.0f;
	microphoneX[2] = MIC_SPACING_METERS * 0.5f;
	microphoneY[2] = MIC_SPACING_METERS * 0.8660254f;
}

void startEvent(uint32_t sampleTime) {
	eventActive = true;
	eventStartTime = sampleTime;
	for (uint8_t mic = 0; mic < 3; mic++) {
		arrivalFound[mic] = false;
		firstArrivalTime[mic] = 0;
	}
}

bool allArrivalsFound() {
	return arrivalFound[0] && arrivalFound[1] && arrivalFound[2];
}

// Solves r1-r0 and r2-r0 using a small Gauss-Newton iteration.
bool calculatePosition(float timeDifference01, float timeDifference02,
											 float &x, float &y) {
	const float distanceDifference01 = SPEED_OF_SOUND_METERS_PER_SECOND * timeDifference01;
	const float distanceDifference02 = SPEED_OF_SOUND_METERS_PER_SECOND * timeDifference02;

	if (fabs(distanceDifference01) > MIC_SPACING_METERS ||
			fabs(distanceDifference02) > MIC_SPACING_METERS) {
		return false;
	}

	// Use a one-metre initial guess in the direction estimated from the TDOAs.
	float directionX = -(timeDifference01 / 0.0002f);
	float directionY = -(timeDifference02 / 0.0002f);
	float directionLength = sqrt(directionX * directionX + directionY * directionY);
	if (directionLength < 0.001f) {
		directionX = 1.0f;
		directionY = 0.0f;
	} else {
		directionX /= directionLength;
		directionY /= directionLength;
	}
	x = directionX;
	y = directionY;

	for (uint8_t iteration = 0; iteration < 20; iteration++) {
		float radius[3];
		for (uint8_t mic = 0; mic < 3; mic++) {
			float dx = x - microphoneX[mic];
			float dy = y - microphoneY[mic];
			radius[mic] = sqrt(dx * dx + dy * dy);
			if (radius[mic] < 0.001f) {
				radius[mic] = 0.001f;
			}
		}

		float residual01 = radius[1] - radius[0] - distanceDifference01;
		float residual02 = radius[2] - radius[0] - distanceDifference02;
		float jacobian[2][2] = {
			{(x - microphoneX[1]) / radius[1] - (x - microphoneX[0]) / radius[0],
			 (y - microphoneY[1]) / radius[1] - (y - microphoneY[0]) / radius[0]},
			{(x - microphoneX[2]) / radius[2] - (x - microphoneX[0]) / radius[0],
			 (y - microphoneY[2]) / radius[2] - (y - microphoneY[0]) / radius[0]}
		};

		float determinant = jacobian[0][0] * jacobian[1][1] -
												jacobian[0][1] * jacobian[1][0];
		if (fabs(determinant) < 0.000001f) {
			return false;
		}

		float stepX = (residual01 * jacobian[1][1] - residual02 * jacobian[0][1]) / determinant;
		float stepY = (jacobian[0][0] * residual02 - jacobian[1][0] * residual01) / determinant;
		x -= stepX;
		y -= stepY;

		if (fabs(stepX) + fabs(stepY) < 0.00001f) {
			return true;
		}
	}

	return true;
}

void logEvent(uint32_t completedTime) {
	float timeDifference01 = (int32_t)(firstArrivalTime[1] - firstArrivalTime[0]) / 1000000.0f;
	float timeDifference02 = (int32_t)(firstArrivalTime[2] - firstArrivalTime[0]) / 1000000.0f;
	float microphoneOriginX = 0.0f;
	float microphoneOriginY = 0.0f;
	bool positionValid = calculatePosition(timeDifference01, timeDifference02,
											 microphoneOriginX, microphoneOriginY);
	float triangleCenterX = (microphoneX[0] + microphoneX[1] + microphoneX[2]) / 3.0f;
	float triangleCenterY = (microphoneY[0] + microphoneY[1] + microphoneY[2]) / 3.0f;
	float x = microphoneOriginX - triangleCenterX;
	float y = microphoneOriginY - triangleCenterY;

	Serial.print("EVENT,");
	Serial.print(millis());
	Serial.print(",");
	Serial.print(firstArrivalTime[0] - eventStartTime);
	Serial.print(",");
	Serial.print(firstArrivalTime[1] - eventStartTime);
	Serial.print(",");
	Serial.print(firstArrivalTime[2] - eventStartTime);
	Serial.print(",");
	Serial.print(positionValid ? x : NAN, 4);
	Serial.print(",");
	Serial.print(positionValid ? y : NAN, 4);
	Serial.print(",");
	Serial.print(positionValid ? sqrt(x * x + y * y) : NAN, 4);
	Serial.print(",");
	Serial.print(positionValid ? atan2(y, x) * 180.0f / PI : NAN, 2);
	Serial.println();

	eventActive = false;
	lastEventTime = completedTime;
}

void readMicrophones() {
	uint32_t sampleTime = micros();
	uint16_t levels[3];

	for (uint8_t mic = 0; mic < 3; mic++) {
		int level = analogRead(MIC_PINS[mic]) - microphoneBias[mic];
		levels[mic] = abs(level);
		bool thresholdCrossed = levels[mic] >= soundThreshold &&
								previousLevel[mic] < soundThreshold;

		if (thresholdCrossed) {
			if (!eventActive && millis() - lastEventTime >= EVENT_COOLDOWN_MS) {
				startEvent(sampleTime);
			}
			if (eventActive && !arrivalFound[mic]) {
				arrivalFound[mic] = true;
				firstArrivalTime[mic] = sampleTime;
			}
		}
		previousLevel[mic] = levels[mic];
	}

	if (eventActive && (allArrivalsFound() || sampleTime - eventStartTime >= MAX_EVENT_LENGTH_US)) {
		if (allArrivalsFound()) {
			logEvent(sampleTime);
		} else {
			eventActive = false;
			lastEventTime = sampleTime;
		}
	}
}

void processSerialCommand() {
	if (!Serial.available()) {
		return;
	}

	String command = Serial.readStringUntil('\n');
	command.trim();
	if (command.startsWith("T=")) {
		int newThreshold = command.substring(2).toInt();
		if (newThreshold > 0 && newThreshold < 2048) {
			soundThreshold = newThreshold;
			Serial.print("Threshold set to ");
			Serial.println(soundThreshold);
		}
	} else if (command.equalsIgnoreCase("STATUS")) {
		Serial.print("STATUS,threshold,");
		Serial.print(soundThreshold);
		Serial.print(",spacing_m,");
		Serial.println(MIC_SPACING_METERS, 3);
	}
}

void setup() {
	Serial.begin(115200);
	// Teensy 4.0 supports 12-bit ADC readings. Other Arduino cores may use
	// their default ADC resolution and should have the threshold retuned.
#if defined(TEENSYDUINO) || defined(ARDUINO_TEENSY40)
	analogReadResolution(12);
#endif
	for (uint8_t mic = 0; mic < 3; mic++) {
		pinMode(MIC_PINS[mic], INPUT);
	}

	setupMicrophoneGeometry();
	calibrateMicrophones();
	Serial.println("EVENT,millis,arrival0_us,arrival1_us,arrival2_us,x_m,y_m,distance_m,angle_deg");
	Serial.println("Position and angle are measured from the triangle center; angle is counter-clockwise from mic 0 toward mic 1.");
	Serial.println("Listening...");
	nextSampleTime = micros();
}

void loop() {
	processSerialCommand();

	uint32_t now = micros();
	if ((int32_t)(now - nextSampleTime) >= 0) {
		nextSampleTime += SAMPLE_INTERVAL_US;
		readMicrophones();
	}
}
