#!/usr/bin/env python3
"""Analysis plots that answer "where did the performance go?".

Usage:  python3 scripts/make_analysis_plots.py
Reads   output/moonshot_ascent.csv, output/moonshot_events.csv
        output/montecarlo.csv                       (optional)
        output/rendezvous.csv, output/rendezvous_tof.csv  (optional)
Writes  output/plots/ascent_budget.png
        output/plots/montecarlo_sensitivity.png
        output/plots/rendezvous.png

Companion to make_plots.py (reentry), make_3d.py (3D views) and
make_mission_plots.py (the phase-by-phase flight suite). Those three all
answer "what did the vehicle do"; this one answers "why", which is a
different set of axes: a delta-v ledger rather than a trajectory, an
input-to-output sensitivity matrix rather than a scatter of samples.

Any input file that is missing is skipped with a note, so this runs after
whichever scripts you have actually flown.
"""
import csv
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "output")
PLOTS = os.path.join(OUT, "plots")
os.makedirs(PLOTS, exist_ok=True)

MU_E = 3.986004418e14
RE = 6371.0088e3

# --- palette (matches make_mission_plots.py, light mode) --------------------
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
BASE = "#c3c2b7"
C_ASC = "#7aa3d6"
C_PARK = "#2a78d6"
C_BURN = "#eb6834"
C_OUT = "#1baf7a"
C_RET = "#9c7400"
C_ENTRY = "#cf3e78"

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "axes.edgecolor": BASE, "axes.labelcolor": INK2,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "xtick.color": MUTED, "ytick.color": MUTED,
    "text.color": INK, "font.size": 10,
    "axes.titlesize": 11, "axes.titleweight": "bold",
    "axes.spines.top": False, "axes.spines.right": False,
    "lines.linewidth": 2.0,
    "font.family": "sans-serif",
})


def read_csv(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None
    cols = {}
    for k in rows[0]:
        try:
            cols[k] = np.array([float(r[k]) if r[k] not in ("", "NaN") else np.nan
                                for r in rows])
        except ValueError:
            cols[k] = np.array([r[k] for r in rows])
    return cols


def maybe(name):
    p = os.path.join(OUT, name)
    return read_csv(p) if os.path.exists(p) else None


def save(fig, name):
    path = os.path.join(PLOTS, name)
    fig.savefig(path, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print("wrote", os.path.relpath(path, ROOT))


def cumtrapz(y, x):
    """Cumulative trapezoid, leading zero, same length as the inputs."""
    return np.concatenate([[0.0], np.cumsum(0.5 * (y[1:] + y[:-1]) * np.diff(x))])


# ===========================================================================
# 1. Ascent delta-v ledger
# ===========================================================================
def ascent_budget():
    asc = maybe("moonshot_ascent.csv")
    if asc is None:
        print("skip ascent_budget: output/moonshot_ascent.csv not found")
        return
    ev = maybe("moonshot_events.csv")

    t = asc["t_s"]
    v = asc["v_rel_ms"]
    gam = np.radians(asc["gamma_deg"])
    m = asc["mass_kg"]
    T = asc["thrust_n"]
    alt = asc["alt_m"]
    g = MU_E / (RE + alt) ** 2

    # The ledger. Ideal delta-v is what the propellant was worth; gravity loss
    # is the component of weight along the flight path; what is left over
    # after subtracting the speed actually gained is everything else the
    # vehicle spent it on — drag, and steering the thrust off the velocity
    # vector, which on a linear-tangent upper stage is most of it.
    a_thrust = T / m
    dv_ideal = cumtrapz(a_thrust, t)
    dv_grav = cumtrapz(g * np.sin(gam), t)
    dv_gained = v - v[0]
    dv_other = dv_ideal - dv_grav - dv_gained

    fig = plt.figure(figsize=(13.2, 7.4))
    gs = fig.add_gridspec(2, 3, height_ratios=[1.25, 1.0], hspace=0.36, wspace=0.42)

    # --- cumulative ledger --------------------------------------------------
    ax = fig.add_subplot(gs[0, :2])
    ax.plot(t, dv_ideal, color=INK2, label="ideal Δv  ∫(T/m)dt")
    ax.plot(t, dv_gained, color=C_PARK, label="speed gained (Earth-relative)")
    ax.fill_between(t, dv_gained, dv_gained + dv_grav, color=C_RET, alpha=0.30,
                    lw=0, label="gravity loss  ∫g·sinγ dt")
    ax.fill_between(t, dv_gained + dv_grav, dv_ideal, color=C_BURN, alpha=0.30,
                    lw=0, label="drag + steering (residual)")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("Δv [m/s]")
    ax.set_title("Where the ascent Δv went")
    ax.legend(loc="upper left", frameon=False, fontsize=9)
    ax.set_xlim(t[0], t[-1])

    # Mark every ascent event the run reported rather than a hardcoded list:
    # separation and ignition events are named after whatever the stages are
    # called, so a renamed or restacked vehicle still gets its own marks.
    if ev is not None and "phase" in ev:
        prev, tier = -1e9, 0
        for name, te in zip(ev["event"][ev["phase"] == "ascent"],
                            ev["t_s"][ev["phase"] == "ascent"]):
            if not (t[0] < te <= t[-1]):
                continue
            # separation and ignition land seconds apart; stagger the labels
            tier = (tier + 1) % 2 if te - prev < 0.05 * (t[-1] - t[0]) else 0
            prev = te
            ax.axvline(te, color=BASE, ls=":", lw=1)
            ax.text(te, dv_ideal[-1] * (0.02 + 0.30 * tier), " " + str(name),
                    color=MUTED, fontsize=8, rotation=90, va="bottom")

    # --- the ledger as one stacked bar --------------------------------------
    ax = fig.add_subplot(gs[0, 2])
    parts = [("speed gained", dv_gained[-1], C_PARK),
             ("gravity", dv_grav[-1], C_RET),
             ("drag + steering", dv_other[-1], C_BURN)]
    base = 0.0
    total = dv_ideal[-1]
    for lab, val, col in parts:
        ax.bar(0, val, bottom=base, width=0.5, color=col, alpha=0.85,
               edgecolor=SURFACE, linewidth=1.5)
        # thin slices get their label outside on a leader, or it lands on top
        # of the neighbouring one
        if val / total > 0.14:
            ax.text(0, base + val / 2, f"{lab}\n{val:.0f} m/s", ha="center",
                    va="center", fontsize=9, color=INK)
        else:
            ax.annotate(f"{lab}  {val:.0f} m/s", xy=(0.25, base + val / 2),
                        xytext=(0.42, base + val / 2), fontsize=9, color=INK2,
                        va="center", ha="left",
                        arrowprops=dict(arrowstyle="-", color=MUTED, lw=1))
        base += val
    ax.text(0, total * 1.03, f"ideal {total:.0f} m/s", ha="center",
            va="bottom", fontsize=9, color=INK2)
    ax.set_xticks([])
    ax.set_xlim(-0.45, 1.5)
    ax.set_ylabel("Δv [m/s]")
    ax.set_title("Ideal Δv, accounted for")
    ax.set_ylim(0, total * 1.14)
    ax.grid(axis="x", visible=False)

    # --- instantaneous loss rates ------------------------------------------
    ax = fig.add_subplot(gs[1, 0])
    ax.plot(t, g * np.sin(gam), color=C_RET, label="gravity  g·sinγ")
    ax.plot(t, a_thrust, color=INK2, label="thrust  T/m")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("acceleration [m/s²]")
    ax.set_title("Loss rate against thrust")
    ax.legend(loc="upper left", frameon=False, fontsize=9)
    ax.set_xlim(t[0], t[-1])

    # --- where each loss accrues -------------------------------------------
    # Two regimes with one shape each: the residual tracks dynamic pressure
    # while there is air, then settles onto the steering loss of the
    # linear-tangent upper stage. The coast between stages shows up as a
    # single-sample spike (no thrust, still losing speed), which is real but
    # would own the whole y-range, so the limits come off the percentiles.
    ax = fig.add_subplot(gs[1, 1])
    rate = np.gradient(dv_other, t)
    ax.plot(t, rate, color=C_BURN)
    ax.axhline(0, color=BASE, lw=1)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("d(drag+steering)/dt [m/s²]")
    ax.set_title("Where the residual accrues")
    ax.set_xlim(t[0], t[-1])
    lo, hi = np.percentile(rate, [1, 99])
    pad = 0.35 * max(hi - lo, 1e-6)
    ax.set_ylim(lo - pad, hi + pad)
    ax2 = ax.twinx()
    ax2.plot(t, asc["qbar_pa"] / 1000, color=C_ASC, lw=1.4, alpha=0.8)
    ax2.set_ylabel("q̄ [kPa]", color=C_ASC, labelpad=1)
    ax2.tick_params(axis="y", colors=C_ASC)
    ax2.grid(False)
    ax2.spines["right"].set_visible(True)
    ax2.spines["right"].set_color(C_ASC)

    # --- the pitch program that bought it -----------------------------------
    ax = fig.add_subplot(gs[1, 2])
    ax.plot(t, np.degrees(gam), color=C_PARK)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("flight-path angle [deg]")
    ax.set_title("Pitch program")
    ax.set_xlim(t[0], t[-1])
    ax.axhline(0, color=BASE, lw=1)

    fig.suptitle("Ascent Δv ledger — pad to parking orbit", fontsize=13,
                 fontweight="bold", y=0.98)
    save(fig, "ascent_budget.png")


# ===========================================================================
# 2. Monte Carlo sensitivity
# ===========================================================================
def montecarlo_sensitivity():
    mc = maybe("montecarlo.csv")
    if mc is None:
        print("skip montecarlo_sensitivity: output/montecarlo.csv not found")
        return

    ins = [("mass_kg", "pod mass"), ("cd_mult", "C_D multiplier"),
           ("rho_mult", "density multiplier"),
           ("trim_alpha_deg", "trim α")]
    outs = [("miss_km", "miss distance"), ("t_flight_s", "flight time"),
            ("v_splash_ms", "splashdown speed"), ("peak_gload", "peak load"),
            ("peak_qdot_wcm2", "peak heat rate"),
            ("heat_load_jcm2", "heat load")]
    ins = [(k, l) for k, l in ins if k in mc and np.nanstd(mc[k]) > 0]
    outs = [(k, l) for k, l in outs if k in mc and np.nanstd(mc[k]) > 0]
    if not ins or not outs:
        print("skip montecarlo_sensitivity: no dispersed columns")
        return

    # `terminated` carries the reason the run ended, not a flag: keep the ones
    # that flew to splashdown, and anything with a finite answer in every
    # column the regression uses
    ntot = len(mc[ins[0][0]])
    ok = np.ones(ntot, bool)
    if "terminated" in mc and mc["terminated"].dtype.kind in "US":
        reason = np.array([str(s).strip().lower() for s in mc["terminated"]])
        if (reason == "splashdown").any():
            ok &= reason == "splashdown"
    for k, _ in ins + outs:
        ok &= np.isfinite(mc[k])
    if ok.sum() < len(ins) + 3:
        print(f"skip montecarlo_sensitivity: only {ok.sum()} usable runs")
        return
    if ok.sum() < ntot:
        print(f"  montecarlo_sensitivity: {ntot - ok.sum()} of {ntot} runs "
              "dropped (did not reach splashdown)")
    X = np.stack([mc[k][ok] for k, _ in ins], 1)
    n = X.shape[0]

    # Standardized regression coefficients: each input's effect with the
    # others held fixed, in units of output sigma per input sigma. With a
    # handful of independent inputs this is the honest version of the
    # one-input-at-a-time scatter — it does not attribute a shared trend
    # twice, and it is directly comparable across rows.
    Xs = (X - X.mean(0)) / X.std(0)
    A = np.column_stack([np.ones(n), Xs])
    beta = np.zeros((len(outs), len(ins)))
    r2 = np.zeros(len(outs))
    for j, (k, _) in enumerate(outs):
        y = mc[k][ok]
        ys = (y - y.mean()) / (y.std() if y.std() > 0 else 1.0)
        coef, *_ = np.linalg.lstsq(A, ys, rcond=None)
        beta[j] = coef[1:]
        resid = ys - A @ coef
        r2[j] = 1 - resid.var() / max(ys.var(), 1e-15)

    fig = plt.figure(figsize=(13.0, 5.6))
    gs = fig.add_gridspec(1, 3, width_ratios=[1.55, 1.0, 1.0], wspace=0.55)

    # --- heatmap ------------------------------------------------------------
    ax = fig.add_subplot(gs[0, 0])
    lim = max(0.25, np.abs(beta).max())
    im = ax.imshow(beta, cmap="RdBu_r", vmin=-lim, vmax=lim, aspect="auto")
    ax.set_xticks(range(len(ins)), [l for _, l in ins], rotation=28, ha="right")
    ax.set_yticks(range(len(outs)), [l for _, l in outs])
    for j in range(len(outs)):
        for i in range(len(ins)):
            ax.text(i, j, f"{beta[j, i]:+.2f}", ha="center", va="center",
                    fontsize=9,
                    color="#ffffff" if abs(beta[j, i]) > lim * 0.55 else INK)
    ax.set_title("Standardized sensitivity  ∂(output σ)/∂(input σ)")
    ax.grid(False)
    # horizontal, under the matrix: a vertical bar lands in the gutter the
    # tornado's category labels need
    cb = fig.colorbar(im, ax=ax, orientation="horizontal", fraction=0.05,
                      pad=0.22)
    cb.outline.set_edgecolor(BASE)

    # noise floor: |beta| below ~2/sqrt(n) is not distinguishable from zero
    floor = 2.0 / np.sqrt(n)

    # --- tornado for the headline output ------------------------------------
    ax = fig.add_subplot(gs[0, 1])
    jm = [k for k, _ in outs].index("miss_km") if "miss_km" in [k for k, _ in outs] else 0
    order = np.argsort(np.abs(beta[jm]))
    ax.barh(range(len(ins)), beta[jm][order],
            color=[C_BURN if b > 0 else C_PARK for b in beta[jm][order]],
            alpha=0.85, height=0.6)
    ax.set_yticks(range(len(ins)), [ins[i][1] for i in order])
    ax.axvline(0, color=BASE, lw=1)
    for s in (-floor, floor):
        ax.axvline(s, color=MUTED, ls=":", lw=1)
    ax.set_xlabel("standardized coefficient")
    ax.set_title(f"Drivers of {outs[jm][1]}  (R² {r2[jm]:.2f})")
    ax.grid(axis="y", visible=False)

    # --- how much of each output the inputs explain -------------------------
    ax = fig.add_subplot(gs[0, 2])
    ax.barh(range(len(outs)), r2, color=C_OUT, alpha=0.8, height=0.6)
    ax.set_yticks(range(len(outs)), [l for _, l in outs])
    ax.set_xlim(0, 1)
    ax.set_xlabel("R² of the linear model")
    ax.set_title("Explained by the dispersions")
    ax.grid(axis="y", visible=False)
    ax.invert_yaxis()

    fig.suptitle(f"Monte Carlo sensitivity — {n} runs "
                 f"(|β| below {floor:.2f} is within the sampling noise)",
                 fontsize=13, fontweight="bold", y=1.01)
    save(fig, "montecarlo_sensitivity.png")


# ===========================================================================
# 3. Rendezvous
# ===========================================================================
def rendezvous():
    rz = maybe("rendezvous.csv")
    if rz is None:
        print("skip rendezvous: output/rendezvous.csv not found "
              "(run: julia --project scripts/run_rendezvous.jl)")
        return
    sw = maybe("rendezvous_tof.csv")

    t = rz["t_s"]
    fig = plt.figure(figsize=(13.2, 5.2))
    gs = fig.add_gridspec(1, 3, width_ratios=[1.25, 1.0, 1.0], wspace=0.46)

    # --- the approach, in the target's RIC frame ----------------------------
    ax = fig.add_subplot(gs[0, 0])
    # along-track on x, radial on y — the convention every prox-ops plot uses,
    # so the target sits at the origin with the chaser closing from behind
    ax.plot(rz["cw_i_m"] / 1e3, rz["cw_r_m"] / 1e3, color=C_PARK,
            label="CW design (linearized)")
    ax.plot(rz["nl_i_m"] / 1e3, rz["nl_r_m"] / 1e3, color=C_BURN, ls="--",
            label="flown (nonlinear)")
    ax.plot(rz["nl_i_m"][0] / 1e3, rz["nl_r_m"][0] / 1e3, "o", color=INK2,
            ms=6, label="chaser at burn 1")
    ax.plot(0, 0, "*", color=C_OUT, ms=15, label="target")
    ax.set_xlabel("along-track [km]")
    ax.set_ylabel("radial [km]")
    ax.set_title("Approach in the target's RIC frame")
    ax.legend(frameon=False, fontsize=9, loc="best")
    ax.axhline(0, color=BASE, lw=1)
    ax.axvline(0, color=BASE, lw=1)
    ax.set_aspect("equal", adjustable="datalim")

    # --- range and range rate ----------------------------------------------
    ax = fig.add_subplot(gs[0, 1])
    rng = rz["range_m"]
    ax.plot(t / 60, rng / 1e3, color=C_BURN)
    ax.set_xlabel("time from burn 1 [min]")
    ax.set_ylabel("range [km]", color=C_BURN)
    ax.tick_params(axis="y", colors=C_BURN)
    ax.set_title("Closing")
    if "range_rate_ms" in rz:
        ax2 = ax.twinx()
        ax2.plot(t / 60, rz["range_rate_ms"], color=C_PARK, lw=1.5)
        ax2.axhline(0, color=BASE, lw=1)
        ax2.set_ylabel("range rate [m/s]", color=C_PARK)
        ax2.tick_params(axis="y", colors=C_PARK)
        ax2.grid(False)
        ax2.spines["right"].set_visible(True)
        ax2.spines["right"].set_color(C_PARK)
    ax.annotate(f"arrival miss {rng[-1]:.0f} m\n"
                f"({100 * rng[-1] / rng[0]:.2f}% of initial range)",
                xy=(t[-1] / 60, rng[-1] / 1e3), xytext=(-10, 30),
                textcoords="offset points", ha="right", fontsize=9, color=INK2,
                arrowprops=dict(arrowstyle="->", color=MUTED, lw=1))

    # --- cost against time of flight ----------------------------------------
    ax = fig.add_subplot(gs[0, 2])
    if sw is not None:
        tof = sw["tof_s"] / 60
        tot = sw["dv_total_ms"]
        fin = np.isfinite(tot)
        # The two-impulse solution blows up as n·tof approaches a multiple of
        # pi — the transfer matrix goes singular, and the burn that has to
        # cover the offset in the remaining time goes to infinity with it.
        # Clip to a few times the cheapest transfer so the basins between the
        # poles stay legible instead of being flattened against one spike.
        cap = 8.0 * np.nanmin(tot[fin])
        ax.plot(tof, np.minimum(tot, cap), color=C_OUT, label="total")
        ax.plot(tof, np.minimum(sw["dv1_ms"], cap), color=C_PARK, lw=1.3,
                label="burn 1")
        ax.plot(tof, np.minimum(sw["dv2_ms"], cap), color=C_BURN, lw=1.3,
                label="burn 2")
        # Mark the poles, and quote the cheapest transfer in the FIRST basin.
        # Taking the global minimum over the sweep would just report wherever
        # the sweep happened to stop — cost keeps falling with every extra
        # revolution you are willing to spend, so "cheapest overall" is a
        # statement about the x-limits, not about the problem.
        over = ~fin | (tot > cap)
        edges = np.where(np.diff(over.astype(int)) == 1)[0]
        first = edges[0] + 1 if len(edges) else len(tot)
        for k in np.where(np.diff(over.astype(int)) == 1)[0]:
            ax.axvline(0.5 * (tof[k] + tof[k + 1]), color=BASE, ls=":", lw=1)
        i = int(np.argmin(np.where(fin[:first], tot[:first], np.inf)))
        ax.plot(tof[i], tot[i], "o", color=INK2, ms=6)
        ax.annotate(f"cheapest before the first\npole: {tot[i]:.2f} m/s "
                    f"at {tof[i]:.0f} min",
                    xy=(tof[i], tot[i]), xytext=(6, 22),
                    textcoords="offset points", fontsize=9, color=INK2,
                    arrowprops=dict(arrowstyle="->", color=MUTED, lw=1))
        ax.set_ylim(0, cap)
        ax.set_xlim(0, tof[-1])
        ax.set_xlabel("time of flight [min]")
        ax.set_ylabel("Δv [m/s]")
        ax.set_title("Cost against transfer time")
        ax.legend(frameon=False, fontsize=9, loc="upper center")
    else:
        ax.text(0.5, 0.5, "output/rendezvous_tof.csv\nnot found",
                ha="center", va="center", color=MUTED, transform=ax.transAxes)
        ax.set_axis_off()

    fig.suptitle("Two-impulse rendezvous — CW design against nonlinear flight",
                 fontsize=13, fontweight="bold", y=1.02)
    save(fig, "rendezvous.png")


if __name__ == "__main__":
    ascent_budget()
    montecarlo_sensitivity()
    rendezvous()
