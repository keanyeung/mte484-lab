# MTE 484 Lab 1 — MATLAB & Simulink Environment Guide

This folder contains the MATLAB scripts and Simulink model for **Lab 1 (Inner Loop Identification and Reference Saturation)**, reproducing the continuous-time inner loop with reference saturation shown in Figure 7(b) of the lab manual.

---

## 1. System & Environment Requirements

To open, simulate, and plot these files without errors, your environment should have:

| Component | Required Version | Notes |
|---|---|---|
| **MATLAB** | **R2024a (v24.1)** or newer | Base MATLAB installation |
| **Simulink** | **R2024a (v24.1)** or newer | Required to open and run `mte484part1e.slx` |
| **Toolboxes** | Simulink Standard Library | Blocks used: *Signal Generator*, *Saturation*, *Sum*, *Gain*, *Transfer Fcn*, *Scope*, *To Workspace* |
| **OS** | macOS, Windows, or Linux | Platform-independent |

### Backwards Compatibility (Older MATLAB Versions)
> [!IMPORTANT]
> The model file `mte484part1e.slx` was built in **MATLAB R2024a**. By default, earlier MATLAB releases (e.g., R2023b, R2023a, R2022b) cannot directly open models saved in newer releases.
> 
> If a collaborator is using an older release:
> 1. In MATLAB R2024a, open `mte484part1e.slx`.
> 2. Go to **File -> Export Model to -> Previous Version...** (or run: `Simulink.exportToVersion('mte484part1e', 'mte484part1e_R2023b.slx', 'R2023b')`).
> 3. Alternatively, they can inspect the saved data directly using `load('SimulatedOutputdata.mat')` and run `mte484part1eplotting.m` without re-simulating.

---

## 2. Included Files

- **`mte484part1esetup.m`**:
  Defines all parameters in the MATLAB base workspace required by the Simulink blocks:
  - `K1 = 1.883` rad/(V·s) — Plant gain identified in Part (e)
  - `tau = 0.0216` s — Motor time constant identified in Part (e)
  - `Kp = 3.5` V/rad — Proportional controller gain
  - `sat = 0.7` rad — Reference saturation limit ($\pm0.7$ rad)
  - `A = 1.0` rad — Square wave input reference amplitude
  - `freq = 0.25` Hz — Square wave frequency (2.0 s half-period)

- **`mte484part1e.slx`**:
  Simulink block diagram implementing Figure 7(b). Logs the following signals to workspace in the `out` object:
  - `out.theta_ref` — Unsaturated square wave ($\pm1.0$ rad)
  - `out.theta_ref_sat` — Saturated reference ($\pm0.7$ rad)
  - `out.theta` — Plant angular position response
  - `out.Vcmd` — Motor control voltage

- **`mte484part1eplotting.m`**:
  Extracts logged data from the `out` simulation object and generates:
  - **Figure 6**: Unsaturated vs. Saturated reference square waves, showing flat-topped clipping at $\pm0.7$ rad in both directions.
  - **Figure 7**: Saturated reference vs. simulated closed-loop plant response $\theta(t)$.

- **`SimulatedOutputdata.mat`**:
  Pre-saved simulation output dataset. Allows regenerating the plots immediately without opening Simulink.

---

## 3. How to Run (Step-by-Step)

### Standard Workflow (Run Simulation):
1. **Open MATLAB** and navigate to this folder (`Matlab_file`).
2. **Initialize Workspace Variables**:
   In the MATLAB Command Window, run:
   ```matlab
   mte484part1esetup
   ```
3. **Simulate the Model**:
   - Open `mte484part1e.slx` and click **Run** (green play button in the toolbar), **OR**
   - Run in the Command Window:
     ```matlab
     out = sim('mte484part1e');
     ```
4. **Generate the Report Plots**:
   In the Command Window, run:
   ```matlab
   mte484part1eplotting
   ```
   This will display **Figure 6** and **Figure 7**.

### Quick Plotting (Without Rerunning Simulink):
If you just need to inspect or export the figures:
```matlab
load('SimulatedOutputdata.mat')
mte484part1eplotting
```

---

## 4. Common Troubleshooting

| Issue | Cause | Solution |
|---|---|---|
| **`Variable 'K1' / 'tau' / 'sat' not found`** | Simulink blocks cannot find the workspace variables. | Run `mte484part1esetup.m` first before clicking Run in Simulink. |
| **`Model was created in a newer version`** | MATLAB release is older than R2024a. | Update to R2024a or export the model to your version using `Simulink.exportToVersion`. |
| **`Dot indexing is not supported for variables of this type` in plotting** | The model hasn't been simulated yet, so `out` does not exist. | Run the simulation first or load `SimulatedOutputdata.mat`. |
