# Three-Microphone Far-Field DOA Detector

This project uses a Teensy 4.0 and three analog microphones to estimate the direction of arrival of a far-field sound source.

The microphones form an equilateral triangle. Each event is detected by a peak-hold envelope, then the buffered waveforms are cross-correlated to estimate the inter-mic delay and convert it to a bearing angle.

The Arduino sketch is in `sound.ino`.

## Hardware geometry

The current code uses these physical positions:

- Mic 0: `(0, 0)`
- Mic 1: `(+0.1625, 0)`
- Mic 2: `(+0.08125, +0.1407)`

All three microphones share ground and are biased near mid-scale ADC voltage so that acoustic energy swings above and below the bias.

The actual pins are defined as:

```cpp
const uint8_t MIC_PINS[3] = {14, 15, 16};
```

## How the detector works

1. The sketch samples all three microphones every `25 us`.
2. It calculates a peak-hold envelope for each channel using a decay factor of `0.995`.
3. When the envelope crosses a calibrated onset threshold, it starts a capture window.
4. A circular buffer stores recent samples from each mic.
5. After the event window ends, it cross-correlates the buffered waveforms over the physically possible lag range.
6. The best lag is refined using parabolic interpolation for sub-sample precision.
7. The resulting inter-mic delays are converted to distance differences using the speed of sound.
8. The bearing is computed from the microphone geometry and is printed over Serial.

The threshold is set during calibration by measuring the ambient noise floor and adding hysteresis and a safety margin.

## Coordinate system and angle convention

The array geometry is based on Mic 0 as the origin. The current code rotates the bearing so that:

- `0°` points directly in front of Mic 0
- `90°` is to the left of that forward direction
- `180°` is behind the array
- `270°` is to the right of the forward direction

This means the result is not a raw Cartesian angle from the +X axis; it is a DOA bearing relative to the forward axis of the array.

The code uses the geometry constants:

```cpp
const float MIC_SPACING_METERS = 0.1625f;
const float SPEED_OF_SOUND_METERS_PER_SECOND = 343.0f;
```

## Calibration

On startup, the sketch performs a quiet-room calibration:

- measures the DC bias of each microphone
- estimates the noise level per channel
- computes the onset threshold used for trigger detection

This is done automatically; the program does not print the calibration values unless you add them back manually.

## Tuning

The main tuning values are:

```cpp
const uint32_t SAMPLE_INTERVAL_US = 25;
const float ENVELOPE_DECAY = 0.995f;
const float NOISE_SIGMA_MULTIPLIER = 8.0f;
const float HYSTERESIS_MARGIN = 15.0f;
```

If the detector triggers too easily in quiet conditions, increase `NOISE_SIGMA_MULTIPLIER`.

## Serial output

Use a Serial Monitor at `115200` baud.

The program emits one line per detected event:

```text
EVENT,angle_deg,42.37
```

The angle is printed in degrees and wraps to `0..359.99`.

## Notes

- The detector is designed for far-field sources, not close-range near-field localization.
- The system uses the physically possible lag range only, which prevents impossible delay estimates.
- It rejects measurements that exceed the physically valid inter-mic spacing by checking the computed geometric constraints.

## Serial Commands

### Show settings

Send:

```text
STATUS
```

The Teensy reports the active threshold and microphone spacing.

### Change the threshold

Send a value from 1 through 2047:

```text
T=250
```

The value is in 12-bit ADC units. A larger value requires a louder sound to trigger an event. A smaller value is more sensitive but may detect background noise.

## Main Settings

These constants and variables control the detector:

| Setting | Purpose |
|---|---|
| `MIC_SPACING_METERS` | Distance between microphone centers |
| `SPEED_OF_SOUND_METERS_PER_SECOND` | Speed used for time-to-distance conversion |
| `soundThreshold` | Current sound detection threshold |
| `SAMPLE_INTERVAL_US` | Time between microphone scans |
| `MAX_EVENT_LENGTH_US` | Maximum time allowed to collect all three arrivals |
| `EVENT_COOLDOWN_MS` | Minimum time between completed events |

## Practical Limitations

This is a simple threshold-crossing detector. Accuracy depends on:

- Microphone spacing measurements.
- Microphone and amplifier response matching.
- Accurate, stable microphone placement.
- The sound being strong enough at all three microphones.
- Reflections and echoes in the room.
- The sound source being reasonably close to the same horizontal plane as the microphones.
- The sampling interval and analog-read timing.

The first threshold crossing is only an approximation of the true sound arrival. For better accuracy later, the project could use synchronized sampling, waveform cross-correlation, filtering, and calibration with known sound-source positions.
