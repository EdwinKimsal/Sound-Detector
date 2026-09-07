# Three-Microphone Sound Detector

This project uses a Teensy 4.0 and three analog microphones to detect a loud sound and estimate where it came from in two dimensions.

The microphones are arranged as an equilateral triangle. When a sound occurs, it reaches the three microphones at slightly different times. Those time differences are used to estimate the sound source position.

The Arduino sketch is in `sound.ino`.

## Hardware Arrangement

The default wiring is:

| Microphone | Teensy pin | Coordinate used by the solver |
|---|---:|---:|
| Mic 0 | A2 | `(0, 0)` |
| Mic 1 | A3 | `(spacing, 0)` |
| Mic 2 | A4 | `(spacing / 2, spacing * 0.866)` |

The default spacing is `0.10` meters. Change `MIC_SPACING_METERS` in the sketch if the physical spacing is different.

All microphones must:

- Share a common ground with the Teensy.
- Produce an analog output.
- Be biased near half of the ADC supply voltage so that sound creates movement above and below the bias level.
- Be mounted in the same horizontal plane as much as possible.

## Overall Process

The program repeats this process:

1. On startup, it samples each microphone while the area is quiet.
2. It averages those samples to find the normal DC bias for each microphone.
3. It continuously reads the three microphones.
4. It subtracts each microphone's bias and takes the absolute value. This gives the sound level around the bias point.
5. When a level crosses `soundThreshold`, a sound event starts.
6. The program records the first threshold-crossing time for each microphone.
7. When all three microphones have detected the event, the program calculates two time differences:

   - Mic 1 arrival time minus Mic 0 arrival time
   - Mic 2 arrival time minus Mic 0 arrival time

8. The time differences become differences in distance using the speed of sound.
9. The program solves for the source position using the known microphone coordinates.
10. The result is shifted from the Mic 0 coordinate system to the center of the triangle.
11. The event is printed as one CSV line over Serial.

If all three microphones do not detect the sound within `MAX_EVENT_LENGTH_US`, the incomplete event is discarded.

## Coordinate System

The calculation internally uses Mic 0 as the origin because this makes the microphone geometry simple.

Before logging, the result is translated to the center of the triangle:

- `x_m = 0, y_m = 0` is the triangle center.
- Positive X points from Mic 0 toward Mic 1.
- Positive Y is toward Mic 2.
- `distance_m` is the distance from the triangle center to the calculated source.
- `angle_deg` is measured from the positive X direction, counter-clockwise.

For example, an angle of `0` degrees points from the triangle center toward Mic 1. An angle of `90` degrees points in the positive Y direction.

## Localization Math

If a sound reaches Mic 1 later than Mic 0, the sound is closer to Mic 0 than Mic 1. The difference in distance is estimated with:

```text
distance difference = speed of sound * time difference
```

The speed of sound is set to `343 m/s` in `SPEED_OF_SOUND_METERS_PER_SECOND`.

For a trial source position `(x, y)`, the distance to each microphone is calculated. The solver compares these distances with the two measured distance differences:

```text
radius1 - radius0 = measured difference 01
radius2 - radius0 = measured difference 02
```

`calculatePosition()` uses a small Gauss-Newton iteration to adjust the trial position until these equations are approximately satisfied.
## Formula Explanation

The localization uses the difference in arrival time between microphones. A single arrival time does not reveal the source distance because the sound could have started at any time. The differences between arrival times remove that unknown start time.

### 1. Microphone Coordinates

Let `s` be `MIC_SPACING_METERS`. Mic 0 is the origin:

```text
Mic 0 = (0, 0)
Mic 1 = (s, 0)
Mic 2 = (s / 2, s * sqrt(3) / 2)
```

For an equilateral triangle, `sqrt(3) / 2` is approximately `0.8660254`. With the default spacing of `0.10 m`, Mic 2 is approximately `(0.05, 0.0866)` meters.

### 2. Arrival-Time Differences

The program records the first threshold-crossing time for each microphone. It then calculates:

```text
timeDifference01 = arrivalTime1 - arrivalTime0
timeDifference02 = arrivalTime2 - arrivalTime0
```

These values are measured in microseconds first, then converted to seconds by dividing by `1,000,000`.

### 3. Convert Time to Distance Difference

Sound travels at approximately `343 m/s`, stored in `SPEED_OF_SOUND_METERS_PER_SECOND`.

```text
distanceDifference01 = 343 * timeDifference01
distanceDifference02 = 343 * timeDifference02
```

For example, if Mic 1 hears the sound `0.0002` seconds after Mic 0:

```text
distanceDifference01 = 343 * 0.0002
                     = 0.0686 meters
```

This means the source is `0.0686 m` closer to Mic 0 than Mic 1.

### 4. Distances from a Trial Position

The solver tests a possible source position `(x, y)`. The distance from that position to each microphone is:

```text
radius0 = sqrt((x - mic0X)^2 + (y - mic0Y)^2)
radius1 = sqrt((x - mic1X)^2 + (y - mic1Y)^2)
radius2 = sqrt((x - mic2X)^2 + (y - mic2Y)^2)
```

The correct source position should satisfy both measured differences:

```text
radius1 - radius0 = distanceDifference01
radius2 - radius0 = distanceDifference02
```

Each equation describes a hyperbola. The source position is where the two hyperbolas intersect.

### 5. Iterative Position Solution

`calculatePosition()` starts with an estimated position and repeatedly improves it. Each iteration:

1. Calculates the three trial distances.
2. Calculates the error in both equations.
3. Estimates how each error changes when `x` or `y` changes. These values form the Jacobian matrix.
4. Solves the two-by-two system for an adjustment to `x` and `y`.
5. Applies that adjustment.

The iteration stops when the adjustment becomes very small or after 20 iterations. This is a small Gauss-Newton solver; it avoids needing a large math library while remaining understandable.

### 6. Move the Result to the Triangle Center

The solver initially returns coordinates relative to Mic 0. The triangle center is the average of the three microphone coordinates:

```text
centerX = (mic0X + mic1X + mic2X) / 3
centerY = (mic0Y + mic1Y + mic2Y) / 3
```

The logged position is then:

```text
centerRelativeX = solverX - centerX
centerRelativeY = solverY - centerY
```

### 7. Distance and Angle from the Center

The distance is the usual two-dimensional distance from the center:

```text
distance_m = sqrt(centerRelativeX^2 + centerRelativeY^2)
```

The angle is calculated with `atan2`, which correctly handles all four quadrants:

```text
angle_deg = atan2(centerRelativeY, centerRelativeX) * 180 / PI
```

The positive X direction points from the triangle center toward Mic 1. Positive angles rotate counter-clockwise toward Mic 2. The result is in degrees.

### 8. Physical Validity Check

The largest possible difference in distance between two microphones is their spacing. Therefore, the program rejects a measurement when either calculated distance difference is larger than `MIC_SPACING_METERS`. This usually indicates noise, an incorrect microphone spacing, or an unreliable threshold crossing.

## Important Functions

### `calibrateMicrophones()`

Reads each microphone 500 times while the area is quiet. The averages are stored in `microphoneBias`. This compensates for differences in microphone modules and their normal DC output.

Run the device with the room quiet during this step.

### `setupMicrophoneGeometry()`

Stores the three microphone coordinates based on the configured equilateral-triangle spacing.

### `startEvent(sampleTime)`

Starts a new sound event and clears the previous arrival-time values.

### `allArrivalsFound()`

Returns `true` only after all three microphones have detected the event.

### `calculatePosition(timeDifference01, timeDifference02, x, y)`

Converts arrival-time differences into distance differences and estimates the source position in the Mic 0 coordinate system.

It returns `false` when the measured timing differences are physically impossible or the calculation becomes unstable.

### `logEvent(completedTime)`

Converts the solver position to triangle-center coordinates, calculates distance and angle, and writes the CSV event record.

### `readMicrophones()`

Performs the main detector work. It reads the microphones, checks for threshold crossings, stores arrival times, and finishes or discards events.

### `processSerialCommand()`

Handles simple commands received over USB Serial.

## Serial Output

Use a Serial Monitor at `115200` baud.

The event header is:

```text
EVENT,millis,arrival0_us,arrival1_us,arrival2_us,x_m,y_m,distance_m,angle_deg
```

A completed event looks like this:

```text
EVENT,12345,0,210,430,0.42,0.18,0.46,23.20
```

The fields are:

| Field | Meaning |
|---|---|
| `millis` | Time since Teensy startup in milliseconds |
| `arrival0_us` | Mic 0 arrival time relative to the event start |
| `arrival1_us` | Mic 1 arrival time relative to the event start |
| `arrival2_us` | Mic 2 arrival time relative to the event start |
| `x_m` | Source X position relative to the triangle center |
| `y_m` | Source Y position relative to the triangle center |
| `distance_m` | Source distance from the triangle center |
| `angle_deg` | Source angle in degrees |

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
