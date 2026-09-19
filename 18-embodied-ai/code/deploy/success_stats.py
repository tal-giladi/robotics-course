"""success_stats.py - how many trials does a robot policy evaluation need? (lessons 18.06, 18.08, 18.10)

Standard library only (runs on the Raspberry Pi too).

  py success_stats.py interval 8 10                 Wilson + exact (Clopper-Pearson) 95% interval
  py success_stats.py compare 17 20 11 20           policy A 17/20 vs policy B 11/20: difference CI + Fisher test
  py success_stats.py plan --p 0.8 --half-width 0.1 trials needed for a +-10 point interval
  py success_stats.py fixed --p0 0.5 --p1 0.8       smallest fixed-n test that separates 50% from 80%
  py success_stats.py sprt --p0 0.5 --p1 0.8 --outcomes 1101111011
                                                    sequential test: stop as soon as the evidence is enough
  py success_stats.py coverage --n 10               why "p +- 1.96*sqrt(p(1-p)/n)" lies for small n
  py success_stats.py peek --p 0.5 --claim 0.5      why "test until the interval looks good" lies
  py success_stats.py demo                          everything above with the lesson's numbers

A "trial" is one attempt at the task from a reset scene; success is a pre-registered, binary
criterion (e.g. "bottle upright on the table, gripper open, within 60 s").
"""
from __future__ import annotations

import argparse
import math
import random
from dataclasses import dataclass
from statistics import NormalDist


def z_for(confidence: float) -> float:
    """Two-sided normal quantile: 0.95 -> 1.95996."""
    return NormalDist().inv_cdf(0.5 + confidence / 2.0)


@dataclass(frozen=True)
class Interval:
    low: float
    high: float

    @property
    def width(self) -> float:
        return self.high - self.low

    def __str__(self) -> str:
        return f"[{100 * self.low:5.1f}%, {100 * self.high:5.1f}%]"


# ----------------------------------------------------------------------------------------------
# One policy: confidence intervals for a success rate
# ----------------------------------------------------------------------------------------------
def wald_interval(k: int, n: int, confidence: float = 0.95) -> Interval:
    """The textbook normal approximation. Shown only to demonstrate that it fails for small n."""
    _check(k, n)
    p = k / n
    h = z_for(confidence) * math.sqrt(p * (1 - p) / n)
    return Interval(max(0.0, p - h), min(1.0, p + h))


def wilson_interval(k: int, n: int, confidence: float = 0.95) -> Interval:
    """Wilson score interval: the recommended default for success rates with n in the tens."""
    _check(k, n)
    z = z_for(confidence)
    p = k / n
    denom = 1 + z * z / n
    center = (p + z * z / (2 * n)) / denom
    half = (z / denom) * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return Interval(max(0.0, center - half), min(1.0, center + half))


def binom_pmf(k: int, n: int, p: float) -> float:
    if p <= 0.0:
        return 1.0 if k == 0 else 0.0
    if p >= 1.0:
        return 1.0 if k == n else 0.0
    return math.comb(n, k) * p**k * (1 - p) ** (n - k)


def binom_cdf(k: int, n: int, p: float) -> float:
    """P(X <= k) for X ~ Binomial(n, p)."""
    if k < 0:
        return 0.0
    if k >= n:
        return 1.0
    return min(1.0, sum(binom_pmf(i, n, p) for i in range(k + 1)))


def clopper_pearson_interval(k: int, n: int, confidence: float = 0.95) -> Interval:
    """Exact (conservative) interval by inverting the binomial CDF with bisection."""
    _check(k, n)
    alpha = 1 - confidence

    def solve(f, lo: float, hi: float) -> float:  # f(lo) and f(hi) have opposite signs
        for _ in range(100):
            mid = (lo + hi) / 2
            if (f(mid) > 0) == (f(lo) > 0):
                lo = mid
            else:
                hi = mid
        return (lo + hi) / 2

    low = 0.0 if k == 0 else solve(lambda p: (1 - binom_cdf(k - 1, n, p)) - alpha / 2, 0.0, 1.0)
    high = 1.0 if k == n else solve(lambda p: binom_cdf(k, n, p) - alpha / 2, 0.0, 1.0)
    return Interval(low, high)


def trials_for_half_width(p: float, half_width: float, confidence: float = 0.95, n_max: int = 100_000) -> int:
    """Smallest n whose Wilson interval around an observed rate p has half-width <= half_width."""
    for n in range(1, n_max + 1):
        k = round(p * n)
        if wilson_interval(k, n, confidence).width / 2 <= half_width:
            return n
    raise ValueError("half-width too small")


def coverage(interval_fn, n: int, p: float, confidence: float = 0.95) -> float:
    """Exact probability that the interval contains the true p (sum over all outcomes)."""
    return sum(
        binom_pmf(k, n, p)
        for k in range(n + 1)
        if interval_fn(k, n, confidence).low <= p <= interval_fn(k, n, confidence).high
    )


# ----------------------------------------------------------------------------------------------
# Two policies
# ----------------------------------------------------------------------------------------------
def newcombe_difference(k1: int, n1: int, k2: int, n2: int, confidence: float = 0.95) -> Interval:
    """CI for p1 - p2 from two Wilson intervals (Newcombe 1998, method 10). Can be negative."""
    p1, p2 = k1 / n1, k2 / n2
    a, b = wilson_interval(k1, n1, confidence), wilson_interval(k2, n2, confidence)
    d = p1 - p2
    low = d - math.sqrt((p1 - a.low) ** 2 + (b.high - p2) ** 2)
    high = d + math.sqrt((a.high - p1) ** 2 + (p2 - b.low) ** 2)
    return Interval(max(-1.0, low), min(1.0, high))


def fisher_exact_two_sided(k1: int, n1: int, k2: int, n2: int) -> float:
    """p-value of Fisher's exact test for the 2x2 table [[k1, n1-k1], [k2, n2-k2]]."""
    total_success, total = k1 + k2, n1 + n2

    def hyper(x: int) -> float:
        return math.comb(n1, x) * math.comb(n2, total_success - x) / math.comb(total, total_success)

    lo, hi = max(0, total_success - n2), min(n1, total_success)
    observed = hyper(k1)
    return min(1.0, sum(hyper(x) for x in range(lo, hi + 1) if hyper(x) <= observed * (1 + 1e-9)))


# ----------------------------------------------------------------------------------------------
# Fixed-n test vs a sequential test (Wald's SPRT)
# ----------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class FixedDesign:
    n: int
    k_min: int          # declare "good policy" if successes >= k_min
    alpha: float        # P(declare good | p = p0)
    power: float        # P(declare good | p = p1)


def fixed_design(p0: float, p1: float, alpha: float = 0.05, beta: float = 0.2, n_max: int = 2000) -> FixedDesign:
    """Smallest n (and threshold) such that a bad policy (p0) passes with prob <= alpha and a good
    policy (p1) passes with prob >= 1 - beta."""
    for n in range(1, n_max + 1):
        for k in range(n + 1):
            a = 1 - binom_cdf(k - 1, n, p0)
            if a <= alpha:
                power = 1 - binom_cdf(k - 1, n, p1)
                if power >= 1 - beta:
                    return FixedDesign(n, k, a, power)
                break  # a larger k only lowers the power
    raise ValueError("no design found")


@dataclass
class SPRT:
    """Wald's sequential probability ratio test for H0: p = p0 ("not good enough") vs H1: p = p1.

    Feed trial outcomes one by one; stop when decision() is not None. Error rates are approximately
    alpha (accept H1 when p = p0) and beta (accept H0 when p = p1)."""

    p0: float
    p1: float
    alpha: float = 0.05
    beta: float = 0.2
    llr: float = 0.0
    n: int = 0
    successes: int = 0

    def __post_init__(self) -> None:
        if not 0 < self.p0 < self.p1 < 1:
            raise ValueError("need 0 < p0 < p1 < 1")

    @property
    def upper(self) -> float:
        return math.log((1 - self.beta) / self.alpha)

    @property
    def lower(self) -> float:
        return math.log(self.beta / (1 - self.alpha))

    def update(self, success: bool) -> str | None:
        self.n += 1
        if success:
            self.successes += 1
            self.llr += math.log(self.p1 / self.p0)
        else:
            self.llr += math.log((1 - self.p1) / (1 - self.p0))
        return self.decision()

    def decision(self) -> str | None:
        if self.llr >= self.upper:
            return "accept H1 (good enough)"
        if self.llr <= self.lower:
            return "accept H0 (not good enough)"
        return None


def simulate_sprt(p_true: float, p0: float, p1: float, alpha: float = 0.05, beta: float = 0.2,
                  runs: int = 20_000, seed: int = 0, n_cap: int = 1000) -> tuple[float, float]:
    """Return (mean trials to a decision, fraction of runs that accepted H1)."""
    rng = random.Random(seed)
    total_n, accepted = 0, 0
    for _ in range(runs):
        t = SPRT(p0, p1, alpha, beta)
        d = None
        while d is None and t.n < n_cap:
            d = t.update(rng.random() < p_true)
        total_n += t.n
        accepted += d is not None and d.startswith("accept H1")
    return total_n / runs, accepted / runs


def peeking_false_claim_rate(p_true: float, claim: float, n_min: int = 5, n_max: int = 50,
                             runs: int = 20_000, seed: int = 0) -> tuple[float, float]:
    """The 'keep testing until it looks good' trap.

    A policy truly succeeds with p_true <= claim. Return (false-claim rate when you check the Wilson
    lower bound once at n_max, false-claim rate when you check after EVERY trial from n_min to n_max and
    stop as soon as the lower bound exceeds `claim`)."""
    rng = random.Random(seed)
    once = peek = 0
    for _ in range(runs):
        k = 0
        claimed = False
        for n in range(1, n_max + 1):
            k += rng.random() < p_true
            if not claimed and n >= n_min and wilson_interval(k, n).low > claim:
                claimed = True
        peek += claimed
        once += wilson_interval(k, n_max).low > claim
    return once / runs, peek / runs


# ----------------------------------------------------------------------------------------------
def _check(k: int, n: int) -> None:
    if n <= 0 or not 0 <= k <= n:
        raise ValueError(f"need 0 <= k <= n and n > 0, got k={k}, n={n}")


def cmd_interval(k: int, n: int, confidence: float) -> None:
    print(f"{k}/{n} successes = {100 * k / n:.0f}%")
    print(f"  Wilson {confidence:.0%} interval:          {wilson_interval(k, n, confidence)}")
    print(f"  Clopper-Pearson (exact) interval: {clopper_pearson_interval(k, n, confidence)}")
    print(f"  Wald (do not use for small n):    {wald_interval(k, n, confidence)}")
    if k == n:
        print(f"  rule of three: true failure rate is below ~{300 / n:.0f}% (95%)")


def cmd_compare(k1: int, n1: int, k2: int, n2: int, confidence: float) -> None:
    d = newcombe_difference(k1, n1, k2, n2, confidence)
    p = fisher_exact_two_sided(k1, n1, k2, n2)
    print(f"A: {k1}/{n1} = {100 * k1 / n1:.0f}%  {wilson_interval(k1, n1, confidence)}")
    print(f"B: {k2}/{n2} = {100 * k2 / n2:.0f}%  {wilson_interval(k2, n2, confidence)}")
    print(f"A - B = {100 * (k1 / n1 - k2 / n2):+.0f} points, {confidence:.0%} CI "
          f"[{100 * d.low:+.1f}, {100 * d.high:+.1f}] points")
    print(f"Fisher exact two-sided p = {p:.3f}  ->  "
          + ("difference is unlikely to be luck" if p < 1 - confidence else "could easily be luck: run more trials"))


def cmd_sprt(p0: float, p1: float, alpha: float, beta: float, outcomes: str) -> None:
    t = SPRT(p0, p1, alpha, beta)
    print(f"SPRT H0: p={p0}  H1: p={p1}  alpha={alpha} beta={beta}  "
          f"bounds: LLR <= {t.lower:.2f} or >= {t.upper:.2f}")
    for ch in outcomes:
        d = t.update(ch == "1")
        print(f"  trial {t.n:2d}: {'success' if ch == '1' else 'failure'}  LLR = {t.llr:+.2f}")
        if d:
            print(f"  -> stop after {t.n} trials: {d}")
            return
    print(f"  -> no decision yet after {t.n} trials: keep testing")


def demo() -> None:
    print("== 1. What does 8/10 tell you? ==")
    cmd_interval(8, 10, 0.95)
    print()
    cmd_interval(10, 10, 0.95)
    print("\n== 2. Same 80%, more trials ==")
    for n in (10, 20, 50, 100):
        print(f"  {int(0.8 * n):3d}/{n:<3d}: Wilson {wilson_interval(int(0.8 * n), n)}")
    print("\n== 3. Trials needed for a +-10 point interval ==")
    for p in (0.5, 0.8, 0.9):
        print(f"  observed ~{p:.0%}: n = {trials_for_half_width(p, 0.10)}")
    print("\n== 4. Is the new policy better? (17/20 vs 11/20) ==")
    cmd_compare(17, 20, 11, 20, 0.95)
    print("\n== 5. Coverage of a nominal 95% interval at n = 10 ==")
    for p in (0.5, 0.8, 0.9, 0.95):
        print(f"  true p = {p:.2f}: Wald {coverage(wald_interval, 10, p):.1%}, "
              f"Wilson {coverage(wilson_interval, 10, p):.1%}")
    print("\n== 6. Fixed-n test vs sequential test, separating 50% from 80% ==")
    fd = fixed_design(0.5, 0.8)
    print(f"  fixed design: n = {fd.n}, pass if successes >= {fd.k_min} "
          f"(false pass {fd.alpha:.1%}, power {fd.power:.1%})")
    for p in (0.5, 0.8):
        asn, acc = simulate_sprt(p, 0.5, 0.8)
        print(f"  SPRT with true p = {p}: mean {asn:.1f} trials, accepts 'good' {acc:.1%} of the time")
    print()
    cmd_sprt(0.5, 0.8, 0.05, 0.2, "1101111011111")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--confidence", type=float, default=0.95)
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("interval"); s.add_argument("k", type=int); s.add_argument("n", type=int)
    s = sub.add_parser("compare")
    for name in ("k1", "n1", "k2", "n2"):
        s.add_argument(name, type=int)
    s = sub.add_parser("plan"); s.add_argument("--p", type=float, default=0.8); s.add_argument("--half-width", type=float, default=0.1)
    s = sub.add_parser("fixed"); s.add_argument("--p0", type=float, required=True); s.add_argument("--p1", type=float, required=True)
    s.add_argument("--alpha", type=float, default=0.05); s.add_argument("--beta", type=float, default=0.2)
    s = sub.add_parser("sprt"); s.add_argument("--p0", type=float, required=True); s.add_argument("--p1", type=float, required=True)
    s.add_argument("--alpha", type=float, default=0.05); s.add_argument("--beta", type=float, default=0.2)
    s.add_argument("--outcomes", required=True, help="string of 1 (success) and 0 (failure), in trial order")
    s = sub.add_parser("coverage"); s.add_argument("--n", type=int, default=10)
    s = sub.add_parser("peek"); s.add_argument("--p", type=float, default=0.5); s.add_argument("--claim", type=float, default=0.5)
    s.add_argument("--n-max", type=int, default=50)
    sub.add_parser("demo")
    a = ap.parse_args()

    if a.cmd == "interval":
        cmd_interval(a.k, a.n, a.confidence)
    elif a.cmd == "compare":
        cmd_compare(a.k1, a.n1, a.k2, a.n2, a.confidence)
    elif a.cmd == "plan":
        print(f"observed ~{a.p:.0%}, target half-width {a.half_width:.0%}: "
              f"n = {trials_for_half_width(a.p, a.half_width, a.confidence)} trials")
    elif a.cmd == "fixed":
        fd = fixed_design(a.p0, a.p1, a.alpha, a.beta)
        print(f"n = {fd.n}, pass if successes >= {fd.k_min}; false pass {fd.alpha:.1%}, power {fd.power:.1%}")
    elif a.cmd == "sprt":
        cmd_sprt(a.p0, a.p1, a.alpha, a.beta, a.outcomes)
    elif a.cmd == "coverage":
        for p in (0.5, 0.7, 0.8, 0.9, 0.95):
            print(f"true p = {p:.2f}: Wald {coverage(wald_interval, a.n, p):.1%}  "
                  f"Wilson {coverage(wilson_interval, a.n, p):.1%}  "
                  f"Clopper-Pearson {coverage(clopper_pearson_interval, a.n, p):.1%}")
    elif a.cmd == "peek":
        once, peek = peeking_false_claim_rate(a.p, a.claim, n_max=a.n_max, runs=5000)
        print(f"true p = {a.p}, claim 'p > {a.claim}': false claims {once:.1%} if you look once at n = {a.n_max}, "
              f"{peek:.1%} if you look after every trial and stop when it looks good")
    else:
        demo()


if __name__ == "__main__":
    main()
