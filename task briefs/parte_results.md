# Lab 1 (e) — Plant Identification: Results

Motor plant `theta(s) / V(s) = K1 / (s (tau s + 1))`, identified from closed-loop step
responses. All numbers below come from 30 recorded steps in
`1e sysid readings/kp15 run1.txt`, `kp20 run1.txt` and `kp25 run1.txt`, reduced by
`analyze_sysid.py` and checked by `validate_model.py`.

**Final answer: K1 = -1.883 ± 0.118 rad/(V·s), tau = 21.6 ± 1.3 ms.**

---

## 1. Method

1. A proportional controller was closed around the motor in `lab1_sysid.ino`:
   `V = clamp(applyStiction(Kp · (theta_ref − theta)), ±5.9 V)`, with the part (d)
   stiction offsets (`STICTION_POS = 0.277 V`, `STICTION_NEG = 0.259 V`) unchanged.
2. The reference was a ±0.1 rad square wave (0.2 rad per step) with a 1 s half period,
   so every period gives one CW and one CCW step and the response settles fully between
   edges (`Ts ≈ 4·(2·tau) ≈ 173 ms` ≪ 1000 ms).
3. Each step was recorded into RAM at the full 1 ms control rate for 600 ms (600 samples)
   and dumped over serial afterwards, so serial throughput never limits the sampling rate.
4. Three gains were used — `Kp = -15, -20, -25 V/rad` — with 10 steps each (5 CW, 5 CCW).
5. For every step, percent overshoot `OS` and time to first peak `Tp` were measured
   (peak refined by a least-squares parabola over ±8 ms), then converted to the
   closed-loop parameters and back to the plant:

   ```
   zeta = -ln(OS/100) / sqrt(pi^2 + ln^2(OS/100))
   wn   = pi / (Tp · sqrt(1 - zeta^2))
   tau  = 1 / (2 · zeta · wn)
   K1   = wn^2 · tau / Kp
   ```

   `OS` is measured against the *measured* steady-state angle `theta_ss`, not against
   `theta_ref`, so any residual stiction offset cannot inflate the overshoot.

---

## 2. Why the loop must be closed

The plant contains a free integrator. In open loop, a constant `V` makes `theta` ramp
without limit at `K1·V` rad/s: there is no steady state, so there is no overshoot and no
peak time to measure, and the gear would run into its travel limit within a fraction of a
second. An open-loop step therefore yields only the ramp slope, which gives `K1` at best
and says nothing about `tau` unless the initial curvature is differentiated out of a noisy,
quantized signal.

Closing a proportional loop turns the plant into a standard second-order system

```
theta(s) / theta_ref(s) = wn^2 / (s^2 + 2·zeta·wn·s + wn^2),
wn^2 = Kp·K1/tau,   2·zeta·wn = 1/tau
```

whose transient shape is fully described by two easily measured quantities, `OS` and `Tp`.
Those two numbers invert uniquely to `zeta` and `wn`, and hence to `K1` and `tau`. Closing
the loop also keeps the gear near zero, inside the linear range of the angle sensor, and
makes the test repeatable and safe.

---

## 3. Sampling rate and its justification

The control interval was set to **1 ms** (`SAMPLE_MS = 1` in `lab1_sysid.ino`), and each
sample's `micros()` timestamp was recorded so the interval could be audited from the data.

Measured from the 17 970 sample-to-sample intervals in the three capture files:

| Quantity | Measured |
|---|---|
| Mean interval | 1000.00 µs |
| Standard deviation | 0.81 µs |
| Intervals within ±1 µs of nominal | 17 944 of 17 970 (99.86 %) |
| Intervals off by more than 11 µs | 18 (0.10 %) |
| Worst single interval | 1042 µs (+42 µs) |

Jitter is therefore negligible: the standard deviation is 0.004 % of `tau`, and even the
worst-case interval is 0.2 % of `tau`.

Why 1 ms is fast enough, and not unnecessarily fast:

- The first peak arrives at `Tp = 73–120 ms` across the three gains, so there are **94
  samples before the peak on average and 73 in the worst case** (`Kp = -25`), far above the
  ~20 needed to locate a peak reliably. The part (d) rate of 20 ms would have given only
  4–6 samples and was unusable.
- One sample period is 4.6 % of `tau` (21.6 ms) and 1.4 % of the shortest `Tp`, so the
  discretization delay is small compared with the dynamics being identified.
- Going faster would shorten the 600-sample RAM buffer below the 600 ms settling window
  without adding resolution where it matters.

**ISR execution time = 33 µs, i.e. 3.3 % of the 1000 µs interval.** The control ISR uses
about one thirtieth of the time available to it, so the 1 kHz rate is sustained with a large
margin — the sampling rate could rise several times over before execution time became the
limit.

The station's Arduino is sealed inside its enclosure, so the A5 oscilloscope pulse
(`digitalWrite(A5, HIGH/LOW)`, still present in the sketch) could not be probed. The same
quantity was instead measured in software: the sketch timestamps the start and end of every
control cycle with `micros()` and reports the mean and worst execution time, plus the
interval between cycles, via the `d` command. Over 3 513 consecutive cycles:

| Quantity | Measured | Meaning |
|---|---|---|
| ISR execution time | mean 33 µs, max 33 µs | 3.3 % of the 1 ms interval; deterministic, since every cycle does the same work |
| Interval between cycles | 984–1016 µs | within ±1.6 % of 1000 µs, so no cycle was ever late or skipped |

The execution figure is what the A5 pulse width would have shown, measured from inside the
enclosure instead of across it. Together with the `micros()` interval statistics from the
captured data (sd 0.81 µs over 17 970 intervals), it confirms no samples were lost at 1 ms.

---

## 4. Stiction verification

The part (d) offsets were carried over unchanged and checked against the brief's
three criteria on all 30 steps. No offset retuning was needed.

| Check | Criterion | Result over 30 steps |
|---|---|---|
| Rounded peak, no flat top | `flat_ratio` < 1.6 | max **1.03** — every peak is within 3 % of an ideal second-order peak width |
| No chatter / limit cycling | no `flat_peak` or `ss_error` warnings | **0 warnings raised** |
| Small steady-state error | \|`ss_err`\| < 0.01 rad | max **0.00090 rad**, mean 0.00027 rad |
| No saturation | `sat = 0` | **0 saturated samples**; max \|V\| = 5.437 V against the 5.9 V clamp |
| Usable steps | — | **30 of 30**, no `exclude` flags |

The worst steady-state error, 0.00090 rad, is 0.45 % of the 0.2 rad step and about 2.5 ADC
counts — the loop parks the gear essentially on the reference, which is exactly what
correct stiction compensation should produce. A flat-topped peak or visible chatter would
have shown as a `flat_peak` warning or a raised `flat_ratio`; neither occurred.

The gear was also observed directly while the loop held each reference level, since the
brief's "no major gear oscillations" criterion is about the hardware rather than the logged
data. A small amount of chatter is audible before each transition, but no sustained
oscillation or limit cycling: the loop parks the gear and holds it. The chatter is expected
from this compensation scheme — the offset is applied whenever the error is nonzero, so
sensor noise of roughly +-0.005 rad flips the command between about +0.3 V and -0.3 V while
the gear is stationary. It is small enough to leave the offsets unchanged. An error deadband
of about +-0.005 rad would remove it if a later lab needs a quieter hold.

---

## 5. Results per gain

From `sysid_summary.csv` (10 steps per gain, mean ± 1 sd):

| Kp [V/rad] | n | OS [%] | Tp [ms] | zeta | wn [rad/s] | K1 [rad/(V·s)] | tau [ms] |
|---|---|---|---|---|---|---|---|
| -15 | 10 | 7.02 ± 1.00 | 110.5 ± 5.9 | 0.646 | 37.4 | **-1.929 ± 0.107** | **20.76 ± 1.32** |
| -20 | 10 | 12.05 ± 1.01 | 93.9 ± 6.3 | 0.559 | 40.5 | **-1.814 ± 0.141** | **22.15 ± 1.09** |
| -25 | 10 | 17.12 ± 0.78 | 77.3 ± 2.9 | 0.490 | 46.7 | **-1.906 ± 0.072** | **21.91 ± 1.05** |
| **All** | **30** | 12.06 | 93.9 | 0.565 | 41.5 | **-1.883 ± 0.118** | **21.61 ± 1.28** |

Split by direction (dir `+` = reference steps up, `theta` increases, gear turns CCW on
`V < 0`; dir `−` = gear turns CW on `V > 0`):

| Direction | Stiction offset used | n | OS [%] | Tp [ms] | K1 | tau [ms] |
|---|---|---|---|---|---|---|
| CCW (`+`) | `STICTION_NEG` = 0.259 V | 15 | 11.51 | 96.1 | -1.832 ± 0.128 | 21.55 ± 1.56 |
| CW (`−`) | `STICTION_POS` = 0.277 V | 15 | 12.62 | 91.7 | -1.934 ± 0.082 | 21.67 ± 0.97 |

**Consistency across gains is the key credibility check.** `K1` and `tau` must not depend
on `Kp`, and they do not: `|K1|` spans only 1.814–1.929 (a 6.1 % spread) and `tau` spans
20.8–22.2 ms (a 6.4 % spread) while `Kp` changes by 67 % and the overshoot changes by a
factor of 2.4. There is no monotonic trend with `Kp` in either parameter.

---

## 6. Final identified plant

```
theta(s) / V(s) = -1.883 / (s (0.0216 s + 1))

K1  = -1.883 ± 0.118 rad/(V·s)     (6.3 % scatter over 30 steps)
tau =  21.61 ± 1.28 ms             (5.9 % scatter over 30 steps)
```

Both sit inside the expected ranges given in the brief: `|K1| = 1.5–3.0 rad/(V·s)` and
`tau = 10–35 ms`.

**Sign.** `K1` is negative because at this station a positive commanded voltage turns the
large gear clockwise, which makes `theta` *decrease* (the part (c) sensor fit has a negative
slope, `THETA_M = -3.645 × 10⁻⁴ rad/count`). A stable proportional loop therefore needs
`Kp < 0` so that the loop gain `Kp·K1` is positive; this is why all three test gains are
negative, and why `wn = sqrt(Kp·K1/tau)` is real.

These values are now recorded in `lab1_motor/lab1_motor.ino` as `PLANT_K1` and `PLANT_TAU`
for Labs 2 and 3.

---

## 7. Model validation

`validate_model.py` simulates the closed-loop step response predicted by the fitted plant at
each step's own `Kp` and compares it sample-by-sample with the measurement. Every step was
checked twice: against the **overall** fit (all 30 steps) and against **its own gain's** fit.
Per-step numbers are in `sysid_validation.csv`; `sysid_overlay.csv` holds the
measured-versus-model traces for plotting the overlay figure.

### Predicted vs measured transient (overall fit)

| Kp | OS measured [%] | OS predicted [%] | Tp measured [ms] | Tp predicted [ms] |
|---|---|---|---|---|
| -15 | 7.0 | 7.3 | 110.5 | 113.1 |
| -20 | 12.1 | 12.3 | 93.9 | 90.4 |
| -25 | 17.1 | 16.6 | 77.3 | 77.5 |
| All | 12.1 | 12.1 | 93.9 | 93.7 |

One fixed `(K1, tau)` reproduces the overshoot to within 0.5 percentage points and the peak
time to within 3.5 ms at all three gains.

### Trajectory error (all 30 steps, 0.201 rad step size)

| Error measure | Overall fit | Per-gain fit |
|---|---|---|
| RMS, model not delayed | 0.0048 rad (2.39 % of step) | 0.0048 rad (2.40 %) |
| RMS, model delayed by the measured dead time | 0.0044 rad (2.18 %) | 0.0044 rad (2.20 %) |
| RMS against the median-filtered angle | **0.0025 rad (1.25 %)** | 0.0026 rad (1.28 %) |
| Peak error, raw measurement | 0.0289 rad | 0.0290 rad |
| Peak error, median-filtered measurement | 0.0116 rad | 0.0118 rad |
| Sensor noise floor (sd of the steady-state tail) | 0.0036 rad | — |

**Reading these numbers honestly.** The raw RMS error of 0.0048 rad is only 1.3× the sensor
noise floor of 0.0036 rad, so most of it is measurement noise rather than model error. Once
single-sample sensor spikes are removed with the same 5-point median filter used in the
analysis, the mismatch drops to **0.0025 rad RMS, or 1.25 % of the 0.2 rad step**. The raw
peak error of 0.029 rad is likewise noise-dominated: its timing is scattered across the whole
600 ms window (mean 205 ms, only 37 % of peaks occurring before `Tp`), which is the signature
of isolated spikes, not of a systematic shape mismatch near the peak. **The fit is good.**

The overall fit and the per-gain fits give errors that agree to within 0.0001 rad. Using one
`(K1, tau)` for all three gains costs essentially nothing, which is independent confirmation
that the plant parameters really are gain-independent.

### Dead time — reported, not hidden

The gear starts moving later than the ideal model does. Each step's delay is measured as

```
dead_ms = (measured onset) − (model onset),   both at 5 % of the step
```

Measured onset averages **10.0 ± 2.5 ms**, but the ideal second-order model itself takes
**8.2 ms** to cover the first 5 % of the step (its response starts with zero slope). The true
unexplained transport delay is therefore only **2.2 ms on average**, not 10 ms. This matters:
quoting the 10 ms onset as "dead time" would overstate the plant's deficiency by a factor of
about 4.5.

That residual 2.2 ms is real backlash and stiction break-away, and it inflates every measured
`Tp` by roughly the same amount, which biases `tau` slightly high. As a rough scale, 2.2 ms
on a mean `Tp` of 93.9 ms is 2.3 %, i.e. of the order of 0.5 ms on the 21.6 ms `tau` — smaller
than the 1.28 ms scatter, so it is not the dominant error. Delaying the model by the measured
dead time lowers the RMS error from 0.0048 to 0.0044 rad, confirming the delay is genuine but
small. The overlay traces in `sysid_overlay.csv` include this per-step delay.

---

## 8. Limitations and honest caveats

1. **Dead time biases `tau` high.** A ~2.2 ms unmodelled delay (backlash plus stiction
   break-away) adds to every `Tp`, so the identified `tau = 21.6 ms` is a slight
   overestimate. The pure second-order model has no delay term, so this error is absorbed
   into `tau`. Correcting for it would move `tau` down by roughly 0.5 ms, within the quoted
   scatter.
2. **CW/CCW asymmetry.** CW steps (`V > 0`, using `STICTION_POS = 0.277 V`) show a higher
   mean overshoot, 12.62 % against 11.51 %, and a more negative `K1`, -1.934 against -1.832
   (a 5.4 % difference). This is consistent with the larger positive-direction stiction
   offset pushing slightly harder through the dead zone, which acts like a small extra loop
   gain in that direction. Both directions were kept in the averages rather than picking the
   more flattering one; the asymmetry is smaller than the ±0.118 overall scatter.
3. **`Kp = -15` is the weakest data point.** It has the smallest overshoot (7.02 %) and by far
   the largest *relative* scatter in overshoot (14.2 %, against 8.4 % at `Kp = -20` and 4.6 %
   at `Kp = -25`), because a small overshoot is measured against the same absolute sensor
   noise. Its `Tp` also scatters most widely (103–120 ms). It was kept because it still sits
   above the 3 % overshoot floor and because agreement across a wide gain range is the point
   of the test, but a lower gain than -15 would not have been usable.
   Note that the largest scatter in `K1` itself is at `Kp = -20` (7.8 %), not at -15 (5.6 %).
4. **One bench session, one run per gain.** All 30 steps came from a single sitting with a
   warm motor. Thermal drift, different gear positions or a re-seated sensor could shift the
   results; nothing here bounds that.
5. **Overshoot measured against `theta_ss`, not `theta_ref`.** This is deliberate and
   removes the stiction offset from the overshoot, but it means `OS` describes the transient
   about the achieved steady state rather than about the commanded one.
6. **ISR timing measured in software, not on a scope.** The enclosure seals the board, so the
   A5 pulse could not be probed; the 33 µs figure comes from `micros()` inside the ISR
   itself (section 3). That timestamp shares its time base with the rest of the sketch, so it
   cannot reveal a fault in the time base — but the independent interval statistics agree
   with it, and a scope would be expected to confirm the same number.
7. **Small-signal result only.** Everything was identified from 0.2 rad steps at 4–5.4 V.
   `K1` and `tau` are not guaranteed to hold for much larger steps, where the command would
   clip against the 5.9 V clamp.

---

## 9. Files behind these numbers

| File | Role |
|---|---|
| `lab1_sysid/lab1_sysid.ino` | P loop, square-wave reference, 1 ms RAM capture |
| `1e sysid readings/kp15 run1.txt`, `kp20 run1.txt`, `kp25 run1.txt` | 30 captured steps |
| `1e sysid readings/analyze_sysid.py` | OS/Tp → zeta, wn → K1, tau; flags bad steps |
| `1e sysid readings/validate_model.py` | Simulates each step from the fit and scores the error |
| `1e sysid readings/sysid_steps.csv` | One row per step with all flags |
| `1e sysid readings/sysid_summary.csv` | Mean ± sd per gain, per direction, overall |
| `1e sysid readings/sysid_validation.csv` | Per-step measured vs simulated, both fits |
| `1e sysid readings/sysid_overlay.csv` | Measured and modelled angle per sample, for the overlay figure |
| `lab1_motor/lab1_motor.ino` | `PLANT_K1`, `PLANT_TAU` recorded for Labs 2 and 3 |

Reproduce with, from `1e sysid readings/`:

```
python analyze_sysid.py
python validate_model.py
python -m unittest test_analyze_sysid test_validate_model
```

## 10. Suggested report figures

1. **Stiction verification** — one captured step plotted with its `V` command, showing a
   rounded peak, no chatter and the gear settling on the reference.
2. **Model overlay** — from `sysid_overlay.csv`, one representative step per gain with
   `theta_measured` and `theta_model` on the same axes (three small plots, or one per gain).
3. **Consistency** — `K1` and `tau` per step plotted against `Kp`, showing no trend.
