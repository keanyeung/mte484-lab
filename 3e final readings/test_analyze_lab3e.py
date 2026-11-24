import unittest

import analyze_lab3e as e


def square_lines(n_half=4, dt=0.01, tau=1.0, offset=0.0, bump_at=None, board=True):
    """Square run starting at 0.10 (the start-up half is not a step). y follows each
    step as a first-order lag; offset shifts it; bump_at=(t0, t1, dy) adds dy in [t0, t1)."""
    lines = ["# start square r=0.100<->0.250 half_period_ms=10000 ctrl=D2c+D1 aw=on dt_ms=1"]
    y, n = 0.10, int(10.0 / dt)
    for h in range(n_half):
        r = 0.10 if h % 2 == 0 else 0.25
        for k in range(n):
            t = (h * n + k) * dt
            y += (r + offset - y) * (dt / tau)
            yy = y + (bump_at[2] if bump_at and bump_at[0] <= t < bump_at[1] else 0.0)
            lines.append(f"{int(round(t * 1000))},{r:.3f},0.1,0.1,0.1,0.5,{yy:.5f}")
        if board and h >= 1:
            lines.append(f"# e step {h} dir={'+' if r == 0.25 else '-'} settle_s=2.00 os_pct=0.0 PASS "
                         f"streak={h} periods={h / 2:.1f}")
    lines.append("# stop, samples=1 saturated=0 aw_held=0 beyond_DA=0 isr_gap_max_us=1010 at_ms=5")
    return lines


class StepTests(unittest.TestCase):
    def test_start_half_not_scored(self):
        steps = e.e_steps(e.parse(square_lines(n_half=3)))
        self.assertEqual([s["dir"] for s in steps], ["+", "-"])

    def test_enter_and_stay_settling(self):
        s = e.e_steps(e.parse(square_lines(n_half=2, tau=1.0)))[0]
        # 0.25 step from 0.10: band 0.240-0.260 entered when 0.15 e^-t = 0.010 -> t = ln(15)
        self.assertAlmostEqual(s["settle_s"], 2.708, delta=0.03)
        self.assertTrue(s["pass"])

    def test_leaving_band_resets_entry(self):
        lines = square_lines(n_half=2, bump_at=(15.0, 15.5, 0.02))   # kicked out of the band mid-step
        s = e.e_steps(e.parse(lines))[0]
        self.assertAlmostEqual(s["settle_s"], 5.5, delta=0.02)
        self.assertTrue(s["pass"])

    def test_late_settling_fails(self):
        s = e.e_steps(e.parse(square_lines(n_half=2, tau=3.4)))[0]   # ln(15)*3.4 = 9.2 s
        self.assertGreater(s["settle_s"], 8.5)
        self.assertFalse(s["pass"])

    def test_ending_outside_band_fails(self):
        s = e.e_steps(e.parse(square_lines(n_half=2, offset=0.012)))[0]
        self.assertIsNone(s["settle_s"])
        self.assertFalse(s["pass"])

    def test_overshoot_fails_at_45_percent(self):
        lines = square_lines(n_half=2, bump_at=(15.0, 15.2, 0.08))  # y ~0.249 + 0.08: 53 % past 0.25
        s = e.e_steps(e.parse(lines))[0]
        self.assertGreaterEqual(s["os_pct"], 45.0)
        self.assertFalse(s["pass"])

    def test_incomplete_last_step_not_scored(self):
        lines = square_lines(n_half=3)
        steps = e.e_steps(e.parse(lines[:-600]))
        self.assertEqual(len([s for s in steps if s["complete"]]), 1)


class StreakTests(unittest.TestCase):
    def test_best_window(self):
        flags = [True] * 7 + [False] + [True] * 21
        steps = [{"pass": f, "complete": True, "gap": False} for f in flags]
        self.assertEqual(e.best_streak(steps), 21)

    def test_gap_step_breaks_streak(self):
        steps = [{"pass": True, "complete": True, "gap": g} for g in [False] * 5 + [True] + [False] * 3]
        self.assertEqual(e.best_streak(steps), 5)

    def test_gap_marks_step(self):
        lines = square_lines(n_half=3)
        del lines[1501:1651]                                  # 1.5 s missing inside the up step
        steps = e.e_steps(e.parse(lines))
        self.assertTrue(steps[0]["gap"])
        self.assertFalse(steps[1]["gap"])


class BoardTests(unittest.TestCase):
    def test_reads_board_lines(self):
        b = e.board_steps(square_lines(n_half=3))
        self.assertEqual([(x["dir"], x["pass"]) for x in b], [("+", True), ("-", True)])
        self.assertEqual(b[0]["settle_s"], 2.0)

    def test_never_settle_on_board(self):
        b = e.board_steps(["# e step 1 dir=+ settle_s=never os_pct=12.0 FAIL streak=0 periods=0.0"])
        self.assertIsNone(b[0]["settle_s"])
        self.assertFalse(b[0]["pass"])

    def test_agreement_counts_pass_flag_mismatches(self):
        ours = [{"dir": "+", "pass": True, "complete": True}, {"dir": "-", "pass": False, "complete": True}]
        board = [{"dir": "+", "pass": True}, {"dir": "-", "pass": True}]
        self.assertEqual(e.disagreements(ours, board), [1])


class PlotTests(unittest.TestCase):
    def test_band_columns(self):
        rows = e.plot_rows(e.parse(square_lines(n_half=2)))
        lo_hi = {(r[1], r[3], r[4]) for r in rows}
        self.assertIn((0.10, 0.094, 0.106), lo_hi)
        self.assertIn((0.25, 0.240, 0.260), lo_hi)


if __name__ == "__main__":
    unittest.main()
