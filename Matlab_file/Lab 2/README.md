# MTE 484 Lab 2 — MATLAB & Simulink Simulation Guide

This folder contains the MATLAB scripts, Simulink models, and simulation datasets for **Lab 2 (Continuous-Time Lead Controller and Discrete-Time Tustin Implementation)**.

---

## 1. System & Environment Requirements

| Component | Required Version | Notes |
|---|---|---|
| **MATLAB** | **R2024a (v24.1)** or newer | Base MATLAB installation |
| **Simulink** | **R2024a (v24.1)** or newer | Required to open and run `.slx` models |
| **Toolboxes** | Control System Toolbox | Used for `tf`, `c2d`, `feedback`, `stepinfo`, `tfdata` |
| **OS** | macOS, Windows, Linux | Platform-independent |

### Backwards Compatibility
> [!NOTE]
> Models are saved in **MATLAB R2024a**. If using an earlier MATLAB release (e.g. R2023b or earlier):
> 1. Export the `.slx` model to an earlier version using `Simulink.exportToVersion`.
> 2. Alternatively, use the standalone plotting scripts which load saved simulation data (`lab2a_data.mat`, `lab2b_data.mat`) without needing to re-run Simulink.

---

## 2. Directory Structure & Included Files

### Part (a) — Continuous-Time Lead Controller
- **`mte484lab2asetup.m`**:
  Initializes plant parameters from Lab 1 ($K_1 = -1.883$, $\tau = 0.0216\text{ s}$), designs continuous lead controller $C_1(s)$ targeting $a=51$, $\sigma=14$, $\omega_d=10$, and sets saturation limit ($0.7\text{ rad}$) and reference input parameters ($A = 1.0\text{ rad}$, $f = 0.5\text{ Hz}$).
- **`mte484lab2parta.slx`**:
  Simulink block diagram implementing the closed-loop system with continuous controller $C_1(s)$, plant transfer function, and input saturation.
- **`mte484lab2parta_run.m`**:
  One-click runner script: executes `mte484lab2asetup.m`, runs `mte484lab2parta.slx` for $t=6\text{ s}$, and saves simulation results to `lab2a_data.mat`.
- **`mte484lab2partaplotting.m`**:
  Loads `lab2a_data.mat`, calculates closed-loop performance (`stepinfo`: settling time, overshoot, steady-state error, peak voltage), and generates Figure 1 (gear angle tracking) and Figure 2 (controller voltage output with $\pm 6\text{ V}$ saturation lines).
- **`lab2a_data.mat`**:
  Pre-simulated workspace data (`out`, `K1`, `tau`, `C1`).

### Part (b) — Discrete-Time Controller (Tustin Discretization)
- **`mte484lab2partbsetup.m`**:
  Runs Part (a) setup, discretizes $C_1(s)$ using Tustin's bilinear approximation at sampling period $T_s = 0.001\text{ s}$ ($1\text{ kHz}$) to obtain $D_1(z) = \text{c2d}(C_1, T_s, \text{'tustin'})$, and outputs discrete transfer function coefficients (`Dnum`, `Dden`).
- **`mte484lab2partb.slx`**:
  Simulink model simulating and comparing both the continuous-time controller and discrete-time controller running side-by-side on the plant.
- **`mte484lab2partb_run.m`**:
  One-click runner script: executes `mte484lab2partbsetup.m`, runs `mte484lab2partb.slx` for $t=6\text{ s}$, and saves output data to `lab2b_data.mat`.
- **`mte484lab2partbplotting.m`**:
  Loads `lab2b_data.mat`, compares continuous vs. discrete performance (settling time, percent overshoot, peak motor voltage), and plots side-by-side comparison figures for gear angle tracking and motor voltage.
- **`lab2b_data.mat`**:
  Pre-simulated comparison dataset (`out`, `C1`, `D1`, `Ts`, `K1`, `tau`).

---

## 3. How to Run the Simulations

### Option A: Complete Run (Setup + Simulation + Plotting)

1. Open MATLAB and set your **Current Folder** to this `Lab 2` directory.
2. For **Part (a)**:
   ```matlab
   mte484lab2parta_run
   mte484lab2partaplotting
   ```
3. For **Part (b)**:
   ```matlab
   mte484lab2partb_run
   mte484lab2partbplotting
   ```

### Option B: Instant Plotting (Without Re-simulating)
If you only want to review or inspect figures without running Simulink:
```matlab
% Plot Part (a)
mte484lab2partaplotting

% Plot Part (b)
mte484lab2partbplotting
```
