# MTE 484 — MATLAB & Simulink Directory

This directory contains MATLAB setup scripts, Simulink simulation models (`.slx`), plotting routines, and pre-computed simulation datasets for **MTE 484**.

---

## Folder Organization

| Directory | Description | Key Topics |
|---|---|---|
| **[`Lab 1/`](./Lab%201)** | Inner Loop Identification & Reference Saturation | Continuous-time motor model, $K_1$, $\tau$, $K_p$, reference saturation |
| **[`Lab 2/`](./Lab%202)** | Continuous & Discrete Controller Design | Lead controller $C_1(s)$, Tustin discretization $D_1(z)$, $1\text{ ms}$ sampling, comparative tracking |
| **[`Lab 3/`](./Lab%203)** | Ball and Beam Outer Loop Control | Continuous lead compensator $C_2(s)$, Tustin discretization $D_2(z)$, ball position tracking & metric evaluation |

---

## Quickstart

1. Clone or pull this repository:
   ```bash
   git pull origin main
   ```
2. In MATLAB, navigate into the corresponding lab directory:
   - For Lab 1: `cd('Matlab_file/Lab 1')`
   - For Lab 2: `cd('Matlab_file/Lab 2')`
   - For Lab 3: `cd('Matlab_file/Lab 3')`
3. Refer to the respective `README.md` inside each lab folder for execution order, required toolboxes, and script details.
