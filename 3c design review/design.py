"""Lab 3 (c) review - design numbers, independent of MATLAB.
Recomputes the groupmate's pole-placement C2(s), its Tustin D2[z], and the
closed-loop poles with the real inner loop (not the unity approximation)."""
import numpy as np

K1, TAU = -1.883, 0.0216
K2, K3 = 0.0609, -4.526
TS = 0.001

# Inner controller C1(s) (Lab 2): -2.408 (s + 71.906)/(s + 32.704), rebuilt from the poles
a, sigma, wd = 51, 14, 10
pc = a + 2 * sigma - 1 / TAU
k = (TAU * (sigma**2 + wd**2 + 2 * a * sigma) - pc) / abs(K1)
zc = TAU * a * (sigma**2 + wd**2) / (abs(K1) * k)
C1N, C1D = np.array([-k, -k * zc]), np.array([1.0, pc])

# Groupmate's outer design: poles -0.9 +- 0.6j, -2, -4 ; C2 = -(a2 s^2 + a1 s + a0)/(s (s + p))
POLES = [-0.9 + 0.6j, -0.9 - 0.6j, -2.0, -4.0]


def outer_design(poles):
    g = -K2 * K3
    pd = np.real(np.poly(poles))
    num = -pd[2:5] / g             # -(a2, a1, a0)
    den = np.array([1.0, pd[1], 0.0])
    return num, den


def tustin(num, den, ts=TS):
    """c2d(..., 'tustin') for a proper rational function, returns (b, a) with a[0] = 1."""
    n = len(den) - 1
    num = np.concatenate([np.zeros(len(den) - len(num)), num])
    kk = 2 / ts
    zm1, zp1 = np.array([1.0, -1.0]), np.array([1.0, 1.0])
    b = np.zeros(n + 1)
    a_ = np.zeros(n + 1)
    for i in range(n + 1):            # coefficient of s^(n-i)
        term = np.array([1.0])
        for _ in range(n - i):
            term = np.polymul(term, kk * zm1)
        for _ in range(i):
            term = np.polymul(term, zp1)
        b = b + num[i] * term
        a_ = a_ + den[i] * term
    return b / a_[0], a_ / a_[0]


def split_coefficients(c2n, c2d, ts=TS):
    """C2 = -[ KP + KI/s + R/(s+p) ], each part Tustin at ts (the sum equals tustin(C2)).
    Difference equations, with e = r - y and e1 = e[k-1]:
        i  = i1 + KI*ts/2*(e + e1)          integrator
        x  = -LA*x1 + LB*(e + e1)           first-order lead R/(s+p)
        u  = -(KP*e + i + x)                gear-angle reference [rad]"""
    a2, a1, a0 = -c2n
    p = c2d[1]
    k = 2 / ts
    r = a1 - a2 * p - a0 / p
    return {"KP": a2, "KI": a0 / p, "LB": r / (k + p), "LA": (p - k) / (k + p), "R": r, "P": p}


def full_loop_poles(c2n, c2d):
    """1 + C2 * T1 * K2 K3 / s^2 = 0 with T1 the true inner closed loop."""
    g_den = np.polymul([TAU, 1.0, 0.0], C1D)          # d1 * s (tau s + 1)
    inner_char = np.polyadd(g_den, K1 * C1N)          # inner closed-loop denominator
    lhs = np.polymul(np.polymul(c2d, [1.0, 0.0, 0.0]), inner_char)
    rhs = K2 * K3 * K1 * np.polymul(c2n, C1N)
    return np.roots(np.polyadd(lhs, rhs))


if __name__ == "__main__":
    np.set_printoptions(precision=15, suppress=False)
    print("C1:", C1N, "/", C1D)
    print("inner closed-loop poles:", np.roots(np.polyadd(np.polymul([TAU, 1, 0], C1D), K1 * C1N)))
    c2n, c2d = outer_design(POLES)
    print("C2 num:", c2n, " den:", c2d)
    a2, a1, a0 = -c2n
    p = c2d[1]
    print(f"split: C2 = -[ {a2:.4f} + {a0/p:.4f}/s + {a1 - a2*p - a0/p:.4f}/(s+{p}) ]")
    sc = split_coefficients(c2n, c2d)
    print("split-form constants for the sketch (Ts = 1 ms):")
    for name in ("KP", "KI", "LB", "LA"):
        print(f"   D2_{name} = {sc[name]:.15g}")
    b, a_ = tustin(c2n, c2d)
    print("D2 num:", repr(b))
    print("D2 den:", repr(a_))
    print("full-loop poles (real inner loop):")
    for r in sorted(full_loop_poles(c2n, c2d), key=lambda z: -z.real):
        print(f"   {r.real:+.4f} {r.imag:+.4f}j")
    # C2 from part (b), for reference
    bb, ab = tustin(np.array([-7.0, -2.45]), np.array([1.0, 2.5]))
    print("part (b) D2:", bb, ab)
