"""Lab 3 (c) review - sampled-data simulation of the full ball-and-beam loop.

1 ms ISR: y -> D2 -> saturator +-0.7 -> D1 -> V (ZOH) -> motor K1/(s(tau s+1)),
integrated at 0.1 ms -> beam K2 -> ball. The ball model is either linear
(K3/s^2, as in Simulink) or a stick-slip ball calibrated from Lab 2(f) and the
Lab 3(b) holds: an offset level angle theta0(y) and a Coulomb band.
Metrics come from analyze_lab3b, the same code that scores the hardware.

Needs numpy. Run from this folder:
    py -3 sim.py 1   (c) D2 vs Simulink, with and without the saturator
    py -3 sim.py 2   part (b) C2 with the stick-slip ball (model check vs run1)
    py -3 sim.py 3   (c) D2: windup vs anti-windup, linear ball
    py -3 sim.py 4   (c) D2: windup vs anti-windup, stick-slip ball
    py -3 sim.py 5   (c) D2 with 0.2 mm sensor noise
"""
import sys
from pathlib import Path

import numpy as np

import design as d

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "3b outer loop readings"))
import analyze_lab3b as an  # noqa: E402

D1B = (-2.454674639351807, 2.284293610869540)
D1A1 = -0.967822458684839
SAT = 0.7
SUB = 10                      # motor/ball substeps per 1 ms sample
H = d.TS / SUB

# Ball disturbance model (gear-angle units, rad). Level angle from the hold
# midpoints: -0.021 at 0.10 m, -0.034 at 0.175 m (linear in y).
THETA0_A, THETA0_SLOPE = -0.021, -0.013 / 0.075


def theta0(y):
    return THETA0_A + THETA0_SLOPE * (y - 0.10)


class LinearD2:
    """Generic Tustin D2 as one difference equation, history UNSATURATED
    (what Simulink's Discrete Transfer Fcn block does)."""
    def __init__(self, b, a):
        self.b, self.a = list(b), list(a)
        self.e = [0.0] * len(b)
        self.u = [0.0] * (len(a) - 1)

    def step(self, e):
        self.e = [e] + self.e[:-1]
        u = sum(bi * ei for bi, ei in zip(self.b, self.e)) - sum(
            ai * ui for ai, ui in zip(self.a[1:], self.u))
        self.u = [u] + self.u[:-1]
        return u


class SplitD2:
    """C2 = -[ a2 + Ki/s + R/(s+p) ], each part Tustin at Ts (the sum is the same D2).
    aw=True: conditional integration - the integrator holds whenever the output
    is saturated and integrating would push it further into saturation."""
    def __init__(self, c2n, c2d, aw):
        sc = d.split_coefficients(c2n, c2d)   # the constants lab3d_outer_loop copies
        self.kp, self.ki, self.lb, self.la = sc["KP"], sc["KI"], sc["LB"], sc["LA"]
        self.aw = aw
        self.i = self.e1 = self.x = 0.0

    def step(self, e):
        i_new = self.i + self.ki * d.TS / 2 * (e + self.e1)
        x_new = -self.la * self.x + self.lb * (e + self.e1)
        u = -(self.kp * e + i_new + x_new)
        if self.aw and abs(u) > SAT and np.sign(-(i_new - self.i)) == np.sign(u):
            i_new = self.i
            u = -(self.kp * e + i_new + x_new)
        self.i, self.x, self.e1 = i_new, x_new, e
        return u


def run(ctrl, t_end=60.0, start_high=True, y0=0.10, ball="linear", noise_mm=0.0,
        a_band=0.011, sat=True, seed=0):
    rng = np.random.default_rng(seed)
    theta = w = 0.0
    y, v = y0, 0.0
    u1 = e11 = 0.0
    hp = 10000
    rows = []
    for k in range(int(round(t_end / d.TS))):
        t = k * d.TS
        high = ((k // hp) % 2 == 0) == start_high
        r = an.Y_HIGH if high else an.Y_LOW
        ym = y + (rng.normal(0, noise_mm * 1e-3) if noise_mm else 0.0)
        th_ref = ctrl.step(r - ym)
        th_sat = max(-SAT, min(SAT, th_ref)) if sat else th_ref
        e1 = th_sat - theta
        u = -D1A1 * u1 + D1B[0] * e1 + D1B[1] * e11
        u1, e11 = u, e1
        V = u
        if k % 10 == 0:
            rows.append((t, r, th_ref, th_sat, theta, V, y))
        for _ in range(SUB):
            w += H * (d.K1 * V - w) / d.TAU
            theta += H * w
            if ball == "linear":
                acc = d.K3 * d.K2 * theta
            else:
                drive = d.K3 * d.K2 * (theta - theta0(y))
                if v == 0.0 and abs(drive) <= a_band:
                    acc = 0.0
                else:
                    sgn = np.sign(v) if v != 0.0 else np.sign(drive)
                    acc = drive - a_band * sgn
            v_new = v + H * acc
            if v != 0.0 and np.sign(v_new) != np.sign(v):
                v_new = 0.0               # friction stops the ball, never reverses it
            v = v_new
            y += H * v
            if y < 0.0127 or y > 0.3693:  # stoppers
                y = min(max(y, 0.0127), 0.3693)
                v = 0.0
    return rows


def report(name, rows):
    steps = an.analyze_run(rows)
    full = [s for s in steps if s["complete"]]
    print(name)
    for s in full:
        seg = [rw for rw in rows if s["t_edge_s"] <= rw[0] < s["t_edge_s"] + 10]
        print(f"    {s['dir']} t={s['t_edge_s']:5.1f}  Ts={an.fmt(s['settling_s'], 2) or '--':>5}"
              f"  OS={s['overshoot_pct']:5.1f}%  err={s['ss_error_mm']:+6.1f} mm"
              f"  sat={s['sat_pct']:4.1f}%  peak|th_ref|={max(abs(rw[2]) for rw in seg):6.2f}"
              f"  peak|V|={max(abs(rw[5]) for rw in seg):.2f}  rest y={seg[-1][6]:.4f}")
    return steps


def first_step(rows, y0=0.10):
    """The t = 0 transition (y0 -> 0.25) when the reference starts high."""
    seg = [rw for rw in rows if rw[0] < 10.0]
    peak = max(rw[6] for rw in seg)
    os_ = max(0.0, peak - 0.25) / (0.25 - y0) * 100
    band = 0.02 * (0.25 - y0)
    out = [i for i, rw in enumerate(seg) if abs(rw[6] - 0.25) > band]
    tset = seg[out[-1] + 1][0] if out and out[-1] < len(seg) - 1 else None
    return os_, tset


if __name__ == "__main__":
    c2n, c2d = d.outer_design(d.POLES)
    b, a_ = d.tustin(c2n, c2d)
    bb, ab = d.tustin(np.array([-7.0, -2.45]), np.array([1.0, 2.5]))
    which = sys.argv[1] if len(sys.argv) > 1 else "1"

    if which == "1":
        rows = run(LinearD2(b, a_), start_high=True)
        print("[1] (c) D2, linear ball, saturator, ref starts HIGH (= mte484lab3partc2.slx): "
              "first step OS %.2f%%  Ts %s" % first_step(rows))
        report("    later steps:", rows)
        rows = run(LinearD2(b, a_), start_high=True, sat=False)
        print("[1b] same, NO saturator: first step OS %.2f%%  Ts %s" % first_step(rows))
        report("    later steps:", rows)
    if which == "2":
        for band in (0.011, 0.003):
            rows = run(LinearD2(bb, ab), start_high=False, ball="stick", a_band=band, t_end=80)
            report(f"[2] part (b) C2, stick-slip ball band={band} m/s^2 (compare run1: up rest 0.205, "
                   f"down rest 0.064 / OS 25%)", rows)
    if which == "3":
        for name, ctrl in (("plain (windup)", LinearD2(b, a_)),
                           ("split, no AW", SplitD2(c2n, c2d, aw=False)),
                           ("split + conditional integration", SplitD2(c2n, c2d, aw=True))):
            report(f"[3] (c) D2 {name}, linear ball, ref starts LOW", run(ctrl, start_high=False))
    if which == "4":
        for name, ctrl in (("plain (windup)", LinearD2(b, a_)),
                           ("split + conditional integration", SplitD2(c2n, c2d, aw=True))):
            report(f"[4] (c) D2 {name}, STICK-SLIP ball, ref starts LOW", run(ctrl, start_high=False,
                                                                              ball="stick", t_end=80))
    if which == "export":
        # Reference traces for the Lab 3(d) overlay. Reference starts HIGH like
        # mte484lab3partc2.slx (ball at 0.10), so the first full period starts at
        # the up edge at 20 s, which analyze_lab3d aligns to the hardware up edges.
        out = Path(__file__).resolve().parent.parent / "3d outer loop readings" / "sim_lab3d.csv"
        out.parent.mkdir(exist_ok=True)
        cols = {}
        for key, ctrl, ball in (("y_aw_off", LinearD2(b, a_), "linear"),
                                ("y_aw_on", SplitD2(c2n, c2d, aw=True), "linear"),
                                ("y_aw_off_stick", LinearD2(b, a_), "stick"),
                                ("y_aw_on_stick", SplitD2(c2n, c2d, aw=True), "stick")):
            rows = run(ctrl, start_high=True, ball=ball)
            cols[key] = [rw[6] for rw in rows]
            cols.setdefault("t_s", [rw[0] for rw in rows])
            cols.setdefault("r", [rw[1] for rw in rows])
            print(f"{key}: first step OS %.2f%%  Ts %s" % first_step(rows))
            report(f"    {key} later steps:", rows)
        names = ["t_s", "r", "y_aw_off", "y_aw_on", "y_aw_off_stick", "y_aw_on_stick"]
        with open(out, "w", newline="") as f:
            f.write(",".join(names) + "\n")
            for k in range(len(cols["t_s"])):
                f.write(",".join(f"{cols[n][k]:.5f}" for n in names) + "\n")
        print(f"wrote {out}")
    if which == "5":
        rows = run(SplitD2(c2n, c2d, aw=True), start_high=False, ball="stick", noise_mm=0.2, t_end=40)
        quiet = [rw for rw in rows if 15 <= rw[0] < 20]
        th = np.array([rw[2] for rw in quiet])
        V = np.array([rw[5] for rw in quiet])
        print(f"[5] noise 0.2 mm: th_ref std at rest {th.std()*1000:.1f} mrad, V std {V.std():.2f} V")
        report("    steps:", rows)
