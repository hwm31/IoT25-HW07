# IoT25-HW07
202334484 손인화

## Photos/videos of the experiment
https://youtube.com/shorts/cgR4GNOFogQ?feature=share

## Explaining my project and Test findings 
"Note: For detailed charts, graphs and comprehensive test information, please refer to the HW7_result.docx file."

### Overview
This project explores distance estimation using Bluetooth Low Energy (BLE) signals by measuring Received Signal Strength Indicator (RSSI) values. The implementation uses a mathematical model to convert RSSI measurements into distance estimates.

### Technical Background
- **RSSI (Received Signal Strength Indicator)**: A measurement of power in received wireless signals, expressed as negative dBm values. Stronger signals are closer to 0 dBm.
- **Distance Formula**: `distance = 10 ^ ((txPower - RSSI) / (10 * n))`
  - Where `txPower` is the calibrated signal strength at 1m
  - `n` is the path loss exponent (varies from 2 in free space to 4 in indoor environments)

### Implementation Parameters
- `txPowerAt1m = -59` (Calibrated RSSI value at 1m distance)
- `PATH_LOSS_EXPONENT = 2.0` (Set for free space conditions)
- `CALIBRATION_FACTOR = 0.4` (Calibration assuming raw measurements needed 2.5x adjustment)

### Test Findings
Tests were conducted at distances of 0.5m, 1m, 2m, 3m, and 4m with the following observations:

#### 0.5m Test
- Despite theoretical expectations, RSSI values at 0.5m were below -59dBm due to environmental interference
- The algorithm performed adequately at this distance despite calibration being set for 1m

#### 1m Test
- Overfitted compared to the original value
- Environmental factors (laptops, people) caused significant path loss and noise
- Resulted in higher than expected measurements

#### 2m Test
- Measured in environment with fewer obstacles
- Relatively less noise compared to 1m measurements
- Some value fluctuations still occurred
- Results were generally appropriate

#### 3m Test
- Measured in space with few obstacles
- Significant variations due to interference from other 2.4GHz devices (router, other BLE devices)
- PATH_LOSS_EXPONENT setting of 2.0 (free space) may not have been ideal

#### 4m Test
- Some measurements jumped above 4m
- Many values were smaller than the original distance
- Weak signal strength (approximately -90dBm) at this distance
- Noise during transmission likely affected readings, resulting in shorter distance calculations

## Suggested Improvements
1. Calculate mean RSSI from larger sample sizes before distance estimation
2. Implement dynamic PATH_LOSS_EXPONENT adjustments:
   - Increase the value for closer distances
   - Decrease the value for farther distances (in locations with few obstacles)

### Error Handling
The implementation returns -1.0 for error cases when:
- RSSI equals 0
- RSSI is greater than or equal to txPower

### Results
The project demonstrates the feasibility of BLE-based distance estimation while highlighting the challenges of environmental interference and signal variability.
