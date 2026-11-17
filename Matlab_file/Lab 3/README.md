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
├── csv_exports/                # Part (c2) simulation data & metric CSV exports
│   ├── lab3_partc2_continuous_performance.csv
│   ├── lab3_partc2_discrete_performance.csv
│   ├── lab3_partc2_summary.csv
│   └── lab3_partc2_timeseries.csv
├── mte484lab3parta.slx          # Part (a) Simulink model (Continuous outer loop + inner loop)
├── mte484lab3partasetup.m      # Part (a) parameters, controller definition, & reference config
├── mte484lab3parta.m          # Part (a) runner script & step response performance analysis
├── mte484lab3partb.slx          # Part (b) Simulink model (Continuous vs Discrete comparison)
├── mte484lab3partbsetup.m      # Part (b) Tustin discretization & coefficient extraction
├── mte484lab3partbrun.m        # Part (b) runner script, comparison plotting, & metric reporting
├── mte484lab3partcsetup.m      # Part (c/c2) improved outer controller pole placement & discretization
├── mte484lab3partc.slx          # Part (c) Simulink model (ball initial position at 0 m)
├── mte484lab3partcrun.m        # Part (c) runner & step response performance analysis
├── mte484lab3partc2.slx         # Part (c2) Simulink model (initial position at 0.10 m via integrators)
├── mte484lab3partcrun2.m       # Part (c2) runner & step response performance analysis
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

### Part (c) & Part (c2) Improved Outer Loop Controller Design
- Controller structure:
  $$C_2(s) = -\frac{a_2 s^2 + a_1 s + a_0}{s(s + p)}$$
- Dominant desired poles placed at $-0.9 \pm 0.6j$ and non-dominant poles at $-2, -4$.
- Discretized via Tustin at $T_s = 0.001\text{ s}$ yielding $D_2(z)$.

#### Part (c) vs. Part (c2) Comparison
The main difference between Part (c) and Part (c2) is the **initial ball position**. In Part (c), the ball starts at 0 m, resulting in a large initial overshoot of approximately 64%. In Part (c2), the ball dynamics are modelled using two integrators, with an initial velocity of 0 m/s and an initial position of 0.10 m. This better represents the intended experimental starting condition and reduces the initial overshoot to approximately 40.18%. Both models use the same controllers and plant parameters, with nearly identical settling times of 6.22 s for the subsequent reference transitions.

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

### Part (c) & Part (c2) Simulation & Analysis
1. Run `mte484lab3partcrun.m` to simulate `mte484lab3partc.slx` (starting from 0 m) and evaluate settling time and overshoot.
2. Run `mte484lab3partcrun2.m` to simulate `mte484lab3partc2.slx` (starting from 0.10 m initial position) and verify the improved initial transient response.
