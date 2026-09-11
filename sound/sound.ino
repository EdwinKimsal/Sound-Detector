// Three-microphone far-field sound Direction-of-Arrival (DOA) detector for Teensy 4.0.
// Mics form an equilateral triangle:
// Mic 0: Origin (0,0)
// Mic 1: Positive X axis (+0.1625, 0)
// Mic 2: Above X axis (+0.08125, +0.1407)
// All microphones must share ground and be biased near half the ADC supply voltage.
//
// REV 2 -----------------------------------------------------------------------------
// The original design measured arrival time as the instant a single raw ADC sample
// crossed a fixed level. Acoustic signals are oscillating (AC) waveforms, and the
// max true inter-mic delay for this array is only ~474us -- comparable to a single
// audio cycle. That made the measured delay extremely sensitive to which half-cycle
// happened to first poke above the threshold, producing wildly different "angles"
// for a source sitting in a fixed position.
//
// This revision:
//   1. Buffers raw samples per mic in a circular buffer.
//   2. Detects event *onset* using a peak-hold envelope (smooths out AC phase, so
//      onset detection no longer depends on which half-cycle happens to be sampled).
//   3. Computes the actual inter-mic delay via cross-correlation of the buffered
//      waveforms (searched only over the physically possible lag range), refined
//      to sub-sample precision with parabolic interpolation of the correlation peak.
// -------------------------------------------------------------------------------------

#include <Arduino.h>
#include <math.h>

const uint8_t MIC_PINS[3] = {14, 15, 16};

const float MIC_SPACING_METERS = 0.1625f;
const float SPEED_OF_SOUND_METERS_PER_SECOND = 343.0f;

const uint32_t SAMPLE_INTERVAL_US = 25;     // 40 kHz per-channel sample rate
const uint32_t EVENT_COOLDOWN_US = 1000000; // ignore new triggers for 1s after an event

// --- Envelope / trigger tuning ---
const float ENVELOPE_DECAY = 0.995f;   // per-sample decay of the peak-hold envelope
const float NOISE_SIGMA_MULTIPLIER = 8.0f;
const float HYSTERESIS_MARGIN = 15.0f;
// If ambient noise still triggers false events, raise NOISE_SIGMA_MULTIPLIER (e.g. 10-12).

// --- Correlation window sizing ---
// Max physically possible inter-mic delay, in samples, with a small safety margin.
const float MAX_INTERMIC_DELAY_SEC = MIC_SPACING_METERS / SPEED_OF_SOUND_METERS_PER_SECOND;
const int16_t MAX_LAG_SAMPLES =
    (int16_t)ceil((MAX_INTERMIC_DELAY_SEC * 1000000.0f) / SAMPLE_INTERVAL_US) + 2; // ~21 samples
const uint16_t CORR_WINDOW_SAMPLES = 64;                                    // ~1.6ms of waveform used
const uint16_t POST_ROLL_SAMPLES = CORR_WINDOW_SAMPLES + MAX_LAG_SAMPLES + 4; // wait time after trigger
const uint16_t BUFFER_LEN = 512; // circular buffer length, comfortably > POST_ROLL_SAMPLES

uint16_t sampleBuf[3][BUFFER_LEN];
uint32_t tickIndex = 0; // ever-increasing logical sample counter

uint16_t micBias[3] = {2048, 2048, 2048};
float envelope[3] = {0, 0, 0};
float onsetThreshold = 120.0f;

enum DetectorState { IDLE, CAPTURING, COOLDOWN };
DetectorState state = IDLE;
uint32_t triggerTick = 0;
uint32_t stateChangeTick = 0;
uint32_t lastEventMicros = 0;

uint32_t nextSampleTime;

inline uint16_t bufIndex(uint32_t tick) {
    return (uint16_t)(tick % BUFFER_LEN);
}

void calibrate() {
    const uint16_t N = 500;
    uint32_t totals[3] = {0, 0, 0};
    float noiseTotal[3] = {0, 0, 0};
    float noiseSqTotal[3] = {0, 0, 0};

    for (uint16_t i = 0; i < N; i++) {
        for (uint8_t m = 0; m < 3; m++) totals[m] += analogRead(MIC_PINS[m]);
        delayMicroseconds(1000);
    }
    for (uint8_t m = 0; m < 3; m++) micBias[m] = totals[m] / N;

    for (uint16_t i = 0; i < N; i++) {
        for (uint8_t m = 0; m < 3; m++) {
            float dev = fabsf((float)analogRead(MIC_PINS[m]) - micBias[m]);
            noiseTotal[m] += dev;
            noiseSqTotal[m] += dev * dev;
        }
        delayMicroseconds(1000);
    }

    float worst = 0;
    for (uint8_t m = 0; m < 3; m++) {
        float mean = noiseTotal[m] / N;
        float var = noiseSqTotal[m] / N - mean * mean;
        float sd = sqrtf(max(var, 0.0f));
        float thresh = mean + NOISE_SIGMA_MULTIPLIER * sd + HYSTERESIS_MARGIN;
        if (thresh > worst) worst = thresh;
    }
    onsetThreshold = min(worst, 1800.0f);
}

// Cross-correlates mic `a` against mic `b` over CORR_WINDOW_SAMPLES ticks starting
// at `windowStartTick`, searching lag in [-MAX_LAG_SAMPLES, +MAX_LAG_SAMPLES].
// Returns the sub-sample lag (in samples) at which mic `a`'s waveform best matches
// mic `b`'s waveform delayed by that amount. Positive = mic `a` arrives later.
float bestLagSamples(uint8_t a, uint8_t b, uint32_t windowStartTick) {
    int32_t bestLag = 0;
    double bestScore = -1e18;

    for (int32_t lag = -MAX_LAG_SAMPLES; lag <= MAX_LAG_SAMPLES; lag++) {
        double sum = 0;
        for (uint16_t i = 0; i < CORR_WINDOW_SAMPLES; i++) {
            uint32_t tickB = windowStartTick + i;
            uint32_t tickA = tickB + lag;
            int16_t va = (int16_t)sampleBuf[a][bufIndex(tickA)] - micBias[a];
            int16_t vb = (int16_t)sampleBuf[b][bufIndex(tickB)] - micBias[b];
            sum += (double)va * (double)vb;
        }
        if (sum > bestScore) {
            bestScore = sum;
            bestLag = lag;
        }
    }

    // Parabolic interpolation around the peak for sub-sample precision.
    double scores[3];
    for (int8_t d = -1; d <= 1; d++) {
        int32_t lag = bestLag + d;
        double sum = 0;
        for (uint16_t i = 0; i < CORR_WINDOW_SAMPLES; i++) {
            uint32_t tickB = windowStartTick + i;
            uint32_t tickA = tickB + lag;
            int16_t va = (int16_t)sampleBuf[a][bufIndex(tickA)] - micBias[a];
            int16_t vb = (int16_t)sampleBuf[b][bufIndex(tickB)] - micBias[b];
            sum += (double)va * (double)vb;
        }
        scores[d + 1] = sum;
    }

    double denom = scores[0] - 2.0 * scores[1] + scores[2];
    double refine = 0.0;
    if (fabs(denom) > 1e-9) {
        refine = 0.5 * (scores[0] - scores[2]) / denom;
        refine = constrain(refine, -1.0, 1.0);
    }
    return (float)bestLag + (float)refine;
}

bool calculateFarFieldBearing(float dt01, float dt02, float &angleDeg) {
    float d01 = SPEED_OF_SOUND_METERS_PER_SECOND * dt01;
    float d02 = SPEED_OF_SOUND_METERS_PER_SECOND * dt02;
    if (fabsf(d01) > MIC_SPACING_METERS || fabsf(d02) > MIC_SPACING_METERS) return false;

    float kx = d01 / MIC_SPACING_METERS;
    float ky = (2.0f * d02 - d01) / (1.7320508f * MIC_SPACING_METERS);

    // Rotate the coordinate system so 0° points directly in front of Mic 0.
    // Mic 0 -> triangle center is the array's forward axis.
    const float FORWARD_OFFSET_RAD = atan2f(0.1407f / 2.0f, 0.1625f / 2.0f);
    float rad = atan2f(ky, kx) - FORWARD_OFFSET_RAD;
    angleDeg = rad * (180.0f / (float)M_PI);
    if (angleDeg < 0.0f) angleDeg += 360.0f;
    return true;
}

void processEvent() {
    uint32_t windowStartTick = triggerTick;

    float lag01 = bestLagSamples(1, 0, windowStartTick); // t1 - t0, in samples
    float lag02 = bestLagSamples(2, 0, windowStartTick); // t2 - t0, in samples

    float dt01 = lag01 * (SAMPLE_INTERVAL_US / 1000000.0f);
    float dt02 = lag02 * (SAMPLE_INTERVAL_US / 1000000.0f);

    float angle;
    if (calculateFarFieldBearing(dt01, dt02, angle)) {
        Serial.print("EVENT,angle_deg,");
        Serial.println(angle, 2);
    }
}

void sampleTick() {
    uint16_t idx = bufIndex(tickIndex);
    bool anyTrigger = false;

    for (uint8_t m = 0; m < 3; m++) {
        uint16_t raw = analogRead(MIC_PINS[m]);
        sampleBuf[m][idx] = raw;
        float dev = fabsf((float)raw - micBias[m]);
        envelope[m] = max(dev, envelope[m] * ENVELOPE_DECAY);
        if (state == IDLE && envelope[m] >= onsetThreshold) anyTrigger = true;
    }

    uint32_t now = micros();
    switch (state) {
        case IDLE:
            if (anyTrigger && (now - lastEventMicros) >= EVENT_COOLDOWN_US) {
                state = CAPTURING;
                triggerTick = tickIndex;
                stateChangeTick = tickIndex;
            }
            break;
        case CAPTURING:
            if (tickIndex - stateChangeTick >= POST_ROLL_SAMPLES) {
                processEvent();
                lastEventMicros = now;
                state = COOLDOWN;
                stateChangeTick = tickIndex;
            }
            break;
        case COOLDOWN:
            if ((tickIndex - stateChangeTick) * SAMPLE_INTERVAL_US >= EVENT_COOLDOWN_US) {
                state = IDLE;
            }
            break;
    }

    tickIndex++;
}

void processSerialCommand() {
    if (!Serial.available()) return;
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.startsWith("T=")) {
        int newThreshold = command.substring(2).toInt();
        if (newThreshold > 0 && newThreshold < 2048) {
            onsetThreshold = newThreshold;
            Serial.print("Threshold manually set to ");
            Serial.println(onsetThreshold);
        }
    } else if (command.equalsIgnoreCase("STATUS")) {
        Serial.printf("STATUS,threshold,%.0f,spacing_m,%.3f,envelope,%.1f,%.1f,%.1f\n",
                      onsetThreshold, MIC_SPACING_METERS, envelope[0], envelope[1], envelope[2]);
    }
}

void setup() {
    Serial.begin(115200);
#if defined(TEENSYDUINO) || defined(ARDUINO_TEENSY40)
    analogReadResolution(12);
#endif
    for (uint8_t m = 0; m < 3; m++) pinMode(MIC_PINS[m], INPUT);

    calibrate();
    nextSampleTime = micros();
}

void loop() {
    processSerialCommand();
    uint32_t now = micros();
    if ((int32_t)(now - nextSampleTime) >= 0) {
        nextSampleTime += SAMPLE_INTERVAL_US;
        sampleTick();
    }
}
