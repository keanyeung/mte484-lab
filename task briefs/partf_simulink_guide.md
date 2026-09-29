# Lab 1 (f) — Simulink model build guide

Written for: the group member building the Simulink half of part (f).

Goal: reproduce Figure 7(b) — the inner loop with a saturator on the reference — and
produce one plot showing the saturator **input and output on the same axes**, clipping in
both the positive and negative directions.

Expect about 15 minutes.

---

## 1. Before you start

Put `sim/lab1_params.m` and your model file in the same folder, then run:

```matlab
run('lab1_params.m')
```

It prints the predicted behaviour and defines the variables the blocks will reference.
Re-run it after any parameter change, and before each simulation if you have cleared the
workspace.

Values it defines:

| Variable | Value | Meaning |
|---|---|---|
| `K1` | 1.883 | rad/(V·s), plant gain from part (e) |
| `tau` | 0.0216 | s, plant time constant from part (e) |
| `plant_num` | `K1` | Transfer Fcn numerator |
| `plant_den` | `[tau 1 0]` | Transfer Fcn denominator, i.e. τs² + s |
| `Kp` | 3.5 | V/rad, proportional gain |
| `ref_sat` | 0.7 | rad, saturator limits |
| `ref_amp` | 1.0 | rad, square wave amplitude |
| `ref_freq_hz` | 0.25 | Hz, square wave frequency (2 s half period) |
| `sim_time` | 12 | s, three full periods |

**On signs:** the measured values are K₁ = −1.883 and K_p = −3.5, both negative. The lab
manual explicitly permits using both as positive numbers in simulation (p. 23). Flipping
both leaves the loop gain K_pK₁ unchanged, so the closed-loop response is identical. Use
positives consistently — mixing one positive with one negative gives an unstable
simulation, which is the most common way to lose an afternoon here.

---

## 2. Blocks to place

| Block | Library | Parameters |
|---|---|---|
| Signal Generator | Sources | Wave form: `square`, Amplitude: `ref_amp`, Frequency: `ref_freq_hz`, Units: `Hertz` |
| Saturation | Discontinuities | Upper limit: `ref_sat`, Lower limit: `-ref_sat` |
| Sum | Math Operations | List of signs: `+-` |
| Gain | Math Operations | Gain: `Kp` |
| Transfer Fcn | Continuous | Numerator: `plant_num`, Denominator: `plant_den` |
| Scope | Sinks | Number of input ports: 3 (see section 4) |
| Mux | Signal Routing | Number of inputs: 3 — optional, if you prefer one scope input |

---

## 3. Wiring

```
Signal Generator ──► Saturation ──►(+) Sum ──► Gain Kp ──► Transfer Fcn ──┬──► theta
                                       (−)▲                               │
                                          └───────────────────────────────┘
```

In words:
1. Signal Generator output → Saturation input. **This wire is the saturator input,
   θ_ref.**
2. Saturation output → Sum `+` input. **This wire is the saturator output, θ_ref,sat.**
3. Sum output → Gain (`Kp`) → Transfer Fcn (the plant) → θ.
4. θ branches back to the Sum `−` input. That branch is the unity feedback.

The plant `K1/(s(τs+1))` expands to `K1/(τs² + s)`, which is why the denominator is
`[tau 1 0]` — coefficients of s², s¹ and s⁰. Getting this wrong is the second most common
error here; a denominator of `[tau 1]` silently gives a first-order plant with no
integrator, and the response will look plausible but wrong (it will settle short of the
reference instead of tracking it exactly).

---

## 4. Scope setup

The brief requires saturator input and output **on the same plot**. Two ways:

- **Three scope inputs** (simplest): Scope → Parameters → Number of input ports = 3, and
  Layout = 1 display so all three share one axis. Connect the saturator input, the
  saturator output, and θ.
- **Mux**: feed all three into a Mux, then one wire to the Scope.

Either way, label the signals so the legend is readable: right-click each wire →
Properties → Signal name (`theta_ref`, `theta_ref_sat`, `theta`). Then in the Scope,
View → Legend.

Set the simulation stop time to `sim_time` (12 s).

---

## 5. What a correct run looks like

| Quantity | Expected | Why |
|---|---|---|
| Saturator input | square wave, ±1.0 rad | The reference you generate |
| Saturator output | square wave, **flat-topped at ±0.7 rad** | The clipping the part is about |
| θ | settles on ±0.7 rad, not ±1.0 | The loop tracks the clipped reference |
| Overshoot | none | ζ = 1.32 with these values, so the response is overdamped |
| Settling | ≈ 0.5 s, from the slow pole at −8.0 s⁻¹ | Well inside the 2 s half period |
| Peak motor voltage | ≈ 4.9 V | K_p × 1.4 rad of error; under the 6 V limit |

`lab1_params.m` prints ζ, the poles and the peak voltage, so you can check the simulation
against them rather than eyeballing it.

**If it doesn't look like that:**

| Symptom | Likely cause |
|---|---|
| θ runs away to ±∞ | Sum signs are `++` instead of `+-`, or one of K₁/K_p is negative while the other is positive |
| θ settles short of ±0.7 with a constant error | Denominator is missing the trailing `0`, so the plant has no integrator |
| Saturator output identical to its input | `ref_amp` ≤ `ref_sat`, or the Saturation limits were typed as numbers that never clip |
| Response oscillates heavily | `Kp` far larger than 3.5 — check the workspace value |
| Blocks show red "undefined variable" | `lab1_params.m` has not been run in this MATLAB session |

---

## 6. Deliverables for the report

1. **Screenshot of the block diagram**, with visible signal labels.
2. **Scope plot** showing saturator input and output on the same axes, over at least one
   full period so both the positive and the negative clip appear. Add a legend and
   cursors marking the ±0.7 rad clip levels — the manual requires legends and cursor
   measurements (p. 15).
3. Save the model as `.slx` in `sim/` — it must be submitted to the Lab 1 dropbox along
   with `lab1_params.m`.

One sentence worth including in the write-up: the limit is ±0.7 rather than ±π/4 = ±0.785
because the Lab 2 controller may overshoot by up to 5%, and 0.7 × 1.05 = 0.735 rad still
sits inside π/4.
