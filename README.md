# Statistical Signal Processing Project

## Course Information
**Course:** Introduction to Statistical Signal Processing (EC5.206)  
**Project Released:** April 23, 2025  
**Due Date:** May 8, 2025

## Project Overview
This project implements adaptive filtering techniques for ECG signal denoising and Kalman filtering for trajectory tracking. The implementation is divided into two main problems with multiple sub-tasks.

---

## Problem 1: Adaptive Filtering for ECG Noise Removal

### Description
Implementation of adaptive filters to remove noise from corrupted ECG signals using a secondary noise reference signal.

### Implemented Algorithms
1. **Steepest Descent Adaptive Filter (SDAF)**
2. **Least Mean Squares (LMS)**
3. **Recursive Least Squares (RLS)**

### Key Parameters
- Filter order: `p = 4`
- LMS step size: `μ = 0.01`
- RLS forgetting factor: `λ = 0.98`

### Input Files Required
- `Signal1.mat` - First corrupted ECG signal
- `Noise1.mat` - First noise reference signal
- `Signal2.mat` - Second corrupted ECG signal
- `Noise2.mat` - Second noise reference signal

### Tasks
- **Task (a):** Apply SDAF, LMS, and RLS filters with order p=4
- **Task (b):** Analyze relative performance in terms of noise removal and convergence
- **Task (c):** Identify non-stationary signal set based on filter performance

### Outputs
- Filtered ECG signals for both signal sets
- Mean Squared Error (MSE) convergence plots
- Performance comparison metrics

---

## Problem 2: Kalman Filter for Trajectory Tracking

### Scenario
Siva drives a car with constant radial velocity (v_r = 10 m/s) and constant angular velocity (v_θ = 3 deg/s) starting from position [1000, 0] meters. Madhuri tracks the car from the origin using noisy range and angle sensors.

### Sensor Characteristics
- **Range sensor variance:** 2500
- **Angle sensor variance:** 16
- **Measurement interval:** 1 second
- **Total duration:** 200 seconds

### Input Files Required
- `range1.mat` - Range measurements from first sensor
- `angle1.mat` - Angle measurements from first sensor
- `range2.mat` - Range measurements from second sensor (Part f)
- `angle2.mat` - Angle measurements from second sensor (Part f)

### Tasks

#### Part (a): Trajectory Plotting
- Plot Siva's true trajectory over 200 seconds
- Animated visualization with time markers

#### Part (b): State Space Model
State vector: `x = [r, θ, v_r, v_θ]`

**State Transition Matrix (F):**
```
F = [1  0  Δt  0 ]
    [0  1  0   Δt]
    [0  0  1   0 ]
    [0  0  0   1 ]
```

**Measurement Matrix (H):**
- Single sensor: `H = [1 0 0 0; 0 1 0 0]`
- Dual sensor: `H = [1 0 0 0; 0 1 0 0; 1 0 0 0; 0 1 0 0]`

#### Part (c): Kalman Filter Equations
Standard Kalman filter prediction and update equations implemented.

#### Part (d): Single Sensor Tracking
- Initial state: `[500, 0, 0, 0]`
- Initial covariance: `diag([1000, 10, 50, 10])`
- Sequential state estimation
- Trajectory and velocity estimation plots
- Investigation of initial covariance effects

#### Part (e): Dual Sensor Configuration
Modified measurement equation to incorporate both sensor sets simultaneously.

#### Part (f): Dual Sensor Tracking
- Implementation with combined sensor measurements
- Performance comparison with single sensor
- Empirical validation of improved accuracy

### Outputs
- True trajectory visualization
- Estimated trajectories (single and dual sensor)
- Estimated velocity profiles
- RMSE and error analysis
- Cumulative mean absolute error plots

---

## File Structure
```
project/
│
├── ques1.m                 # Problem 1 implementation
├── ques2.m                 # Problem 2 implementation
│
├── Signal1.mat             # Input: Corrupted ECG signal 1
├── Noise1.mat              # Input: Noise reference 1
├── Signal2.mat             # Input: Corrupted ECG signal 2
├── Noise2.mat              # Input: Noise reference 2
│
├── range1.mat              # Input: Range sensor 1 measurements
├── angle1.mat              # Input: Angle sensor 1 measurements
├── range2.mat              # Input: Range sensor 2 measurements
├── angle2.mat              # Input: Angle sensor 2 measurements
│
└── project statement.pdf   # Project specifications
```

---

## Usage Instructions

### Running Problem 1
```matlab
% Open MATLAB and navigate to project directory
run('ques1.m')
```
This will:
- Load signal and noise data
- Apply SDAF, LMS, and RLS filters
- Generate filtered signal plots
- Display MSE convergence curves
- Print performance metrics

### Running Problem 2
```matlab
% Open MATLAB and navigate to project directory
run('ques2.m')
```
This will:
- Animate Siva's true trajectory
- Perform single-sensor Kalman filtering
- Perform dual-sensor Kalman filtering
- Generate trajectory comparison plots
- Display velocity estimation plots
- Print RMSE comparison metrics

---

## Key Results and Metrics

### Problem 1 Metrics
- Mean Squared Error (MSE) for each filter
- Convergence speed comparison
- Stationarity analysis

### Problem 2 Metrics
- RMSE for radial distance estimation
- RMSE for radial velocity estimation
- RMSE for angular velocity estimation
- Minimum error values
- Cumulative mean absolute error trends

---

## Requirements
- MATLAB R2018b or later
- Signal Processing Toolbox (recommended)
- All input `.mat` files in the same directory

---

## Notes
- Ensure all data files are in the working directory before running scripts
- Figures are generated automatically and can be saved manually
- Initial covariance matrix can be modified in `ques2.m` for sensitivity analysis
- Animation speed can be adjusted by modifying the loop delay in Part (a)

---

## Author
Ritama Sanyal - 2023112027

---

## References
- Course lecture notes on adaptive filtering
- Kalman filtering fundamentals
- Statistical signal processing principles
