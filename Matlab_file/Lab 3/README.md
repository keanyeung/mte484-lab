# MTE 484 — Lab 3: Ball and Beam Outer Loop Control

This directory contains the MATLAB scripts and Simulink models for **Lab 3** of MTE 484 (Outer-Loop Design and Discretization for the Ball and Beam system).

---

## 1. Overview

Lab 3 builds on the inner gear-position loop developed in Lab 2 to control the ball position along the beam (outer loop):
- **Plant Dynamics**:
  - $K_2 = 0.0609$ (gear angle to beam angle)
  - $K_3 = -4.526\text{ m/s}^2/\text{rad}$ (beam angle to ball acceleration)
  - Combined dynamics: $G_{\theta \to y}(s) = \frac{K_2 K_3}{s^2} = \frac{-0.2756}{s^2}$
- **Actuator Limits**: Gear-angle saturation at $\pm 0.7\text{ rad}$ ($\approx \pm 40^\circ$).
- **Reference**: Square pulse toggling between $0.10\text{ m}$ and $0.25\text{ m}$ (period $20\text{ s}$, duty cycle $50\%$).

---

## 2. Directory Structure

```text
Lab 3/
├── mte484lab3parta.slx          # Part (a) Simulink model (Continuous outer loop + inner loop)
├── mte484lab3partasetup.m      # Part (a) parameters, controller definition, & reference config
├── mte484lab3parta.m          # Part (a) runner script & step response performance analysis
├── mte484lab3partb.slx          # Part (b) Simulink model (Continuous vs Discrete comparison)
├── mte484lab3partbsetup.m      # Part (b) Tustin discretization & coefficient extraction
├── mte484lab3partbrun.m        # Part (b) runner script, comparison plotting, & metric reporting
└── README.md                    # Lab 3 documentation
```

---

## 3. Controller Specifications

### Part (a) Continuous Outer Loop Controller
- Continuous lead compensator:
  $$C_2(s) = -7 \frac{s + 0.35}{s + 2.5} = \frac{-7s - 2.45}{s + 2.5}$$
- Numerator: `[-7, -2.45]`
- Denominator: `[1, 2.5]`

### Part (b) Discrete Outer Loop Controller
- Sampling period: $T_s = 0.001\text{ s}$ ($1\text{ kHz}$)
- Discretization method: **Tustin** (bilinear approximation) via `c2d(C2, Ts, 'tustin')`.
- Discrete transfer function $D_2(z)$ coefficients are extracted using `tfdata(D2, 'v')` for digital implementation on the real-time target.

---

## 4. How to Run

### Part (a) Continuous Simulation & Analysis
1. Open MATLAB and set current folder to `Lab 3`.
2. Run `mte484lab3partasetup.m` to load plant parameters and controller into the workspace.
3. Run `mte484lab3parta.m` to simulate `mte484lab3parta.slx`, plot reference tracking, and compute step metrics (settling time, overshoot, steady-state error).

### Part (b) Continuous vs. Discrete Comparison
1. Run `mte484lab3partbrun.m` directly.
2. The script will automatically:
   - Call `mte484lab3partbsetup.m` to compute discrete coefficients.
   - Simulate `mte484lab3partb.slx` for 60 seconds.
   - Plot continuous vs. discrete ball position trajectories.
   - Print performance metrics and maximum error between continuous and discrete implementations.
