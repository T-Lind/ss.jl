#!/usr/bin/env python3
"""Render verification plots from the simulation CSV outputs.

Usage:  python3 scripts/make_plots.py
Reads   output/nominal_trajectory.csv, output/nominal_events.csv,
        output/montecarlo.csv
Writes  output/plots/*.png

(The Julia-native equivalent using Plots.jl is scripts/make_plots.jl —
this Python version exists so plots can be produced with only matplotlib.)
"""
import csv
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "output")
PLOTS = os.path.join(OUT, "plots")
os.makedirs(PLOTS, exist_ok=True)

# --- palette (validated reference palette, light mode) ----------------------
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
BASE = "#c3c2b7"
S1, S2, S3 = "#2a78d6", "#eb6834", "#1baf7a"  # categorical slots 1-3

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
    cols = {}
    for k in rows[0]:
        try:
            cols[k] = np.array([float(r[k]) for r in rows])
        except ValueError:
            cols[k] = np.array([r[k] for r in rows])
    return cols


traj = read_csv(os.path.join(OUT, "nominal_trajectory.csv"))
events = read_csv(os.path.join(OUT, "nominal_events.csv"))
mc = read_csv(os.path.join(OUT, "montecarlo.csv"))

EV = {n: i for i, n in enumerate(events["event"])}
t_ei = events["t_s"][EV["entry_interface"]]
TARGET = (32.5, -121.5)

entry = traj["t_s"] >= t_ei - 20  # entry-phase mask, small pre-EI margin


def style(ax):
    ax.grid(True, which="major", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def event_lines(ax, names=("deploy_drogue", "deploy_main")):
    """Vertical markers on axes whose x is minutes from EI."""
    for n in names:
        if n in EV:
            ax.axvline((events["t_s"][EV[n]] - t_ei) / 60, color=BASE,
                       linewidth=1.0, linestyle=":")


# ============================ 1. flight profile ==============================
fig, axs = plt.subplots(2, 2, figsize=(11, 7.5))
fig.suptitle("Nominal reentry — flight profile", fontweight="bold", color=INK)

ax = axs[0, 0]
ax.plot(traj["t_s"] / 60, traj["alt_m"] / 1e3, color=S1)
for n, lab, off in [("entry_interface", "EI", (8, 6)), ("deploy_drogue", "drogue", (-14, 14)),
                    ("deploy_main", "main", (6, -14)), ("splashdown", "splash", (8, 6))]:
    i = EV[n]
    ax.plot(events["t_s"][i] / 60, events["alt_m"][i] / 1e3, "o", ms=6, color=S2, zorder=5)
    ax.annotate(lab, (events["t_s"][i] / 60, events["alt_m"][i] / 1e3),
                textcoords="offset points", xytext=off, fontsize=9, color=INK2)
ax.set_xlabel("time [min]"); ax.set_ylabel("geodetic altitude [km]")
ax.set_title("Altitude vs time")

ax = axs[0, 1]
ax.plot(traj["v_rel_ms"][entry] / 1e3, traj["alt_m"][entry] / 1e3, color=S1)
ax.set_xlabel("relative velocity [km/s]"); ax.set_ylabel("altitude [km]")
ax.set_title("Velocity–altitude (entry)")
ax.axhline(120, color=BASE, linewidth=1.0, linestyle=":")
ax.annotate("EI 120 km", (ax.get_xlim()[1] * 0.72, 122), fontsize=9, color=MUTED)

ax = axs[1, 0]
te = (traj["t_s"][entry] - t_ei) / 60
ax.plot(te, traj["gload"][entry], color=S1)
i = np.argmax(traj["gload"])
ax.annotate(f"peak {traj['gload'][i]:.1f} g",
            ((traj["t_s"][i] - t_ei) / 60, traj["gload"][i]),
            textcoords="offset points", xytext=(10, -2), fontsize=9, color=INK2)
ax.set_xlabel("time from EI [min]"); ax.set_ylabel("sensed deceleration [g]")
ax.set_title("Aerodynamic g-load")

ax = axs[1, 1]
ax.plot(te, traj["qbar_pa"][entry] / 1e3, color=S1)
i = np.argmax(traj["qbar_pa"])
ax.annotate(f"peak {traj['qbar_pa'][i]/1e3:.1f} kPa",
            ((traj["t_s"][i] - t_ei) / 60, traj["qbar_pa"][i] / 1e3),
            textcoords="offset points", xytext=(10, -2), fontsize=9, color=INK2)
ax.set_xlabel("time from EI [min]"); ax.set_ylabel("dynamic pressure [kPa]")
ax.set_title("Dynamic pressure")

for a in axs.flat:
    style(a)
fig.tight_layout()
fig.savefig(os.path.join(PLOTS, "nominal_profile.png"), dpi=150)
plt.close(fig)

# ============================ 2. aerothermal =================================
fig, axs = plt.subplots(1, 3, figsize=(13, 4.2))
fig.suptitle("Nominal reentry — aerothermal environment", fontweight="bold", color=INK)

ax = axs[0]
ax.plot(te, traj["qdot_conv_wcm2"][entry], color=S1, label="convective (Sutton–Graves)")
ax.plot(te, traj["qdot_rad_wcm2"][entry], color=S2, label="radiative (Tauber–Sutton)")
ax.legend(frameon=False, fontsize=9)
ax.set_xlabel("time from EI [min]"); ax.set_ylabel("stagnation heat flux [W/cm²]")
ax.set_title("Stagnation heating rate")

ax = axs[1]
ax.plot(traj["qdot_conv_wcm2"][entry] + traj["qdot_rad_wcm2"][entry],
        traj["alt_m"][entry] / 1e3, color=S1)
i = np.argmax(traj["qdot_conv_wcm2"] + traj["qdot_rad_wcm2"])
ax.plot(traj["qdot_conv_wcm2"][i] + traj["qdot_rad_wcm2"][i], traj["alt_m"][i] / 1e3,
        "o", ms=6, color=S2, zorder=5)
ax.annotate(f"peak {traj['qdot_conv_wcm2'][i]+traj['qdot_rad_wcm2'][i]:.0f} W/cm²\n"
            f"at {traj['alt_m'][i]/1e3:.0f} km",
            (traj["qdot_conv_wcm2"][i], traj["alt_m"][i] / 1e3),
            textcoords="offset points", xytext=(10, 8), fontsize=9, color=INK2)
ax.set_xlabel("total stagnation heat flux [W/cm²]"); ax.set_ylabel("altitude [km]")
ax.set_title("Heat flux vs altitude")

ax = axs[2]
ax.plot(te, traj["heat_load_jcm2"][entry] / 1e3, color=S1)
ax.set_xlabel("time from EI [min]"); ax.set_ylabel("integrated heat load [kJ/cm²]")
ax.set_title("Stagnation heat load")

for a in axs.flat:
    style(a)
fig.tight_layout()
fig.savefig(os.path.join(PLOTS, "nominal_heating.png"), dpi=150)
plt.close(fig)

# ============================ 3. attitude (4th DOF) ==========================
fig, axs = plt.subplots(1, 3, figsize=(13, 4.2))
fig.suptitle("Nominal reentry — pitch (body-angle) degree of freedom",
             fontweight="bold", color=INK)

ax = axs[0]
ax.plot(te, traj["alpha_deg"][entry], color=S1)
ax.set_xlabel("time from EI [min]"); ax.set_ylabel("angle of attack α [deg]")
ax.set_title("AoA oscillation & damping")
event_lines(ax)

ax = axs[1]
ax.plot(te, traj["pitch_rate_dps"][entry], color=S1)
ax.set_xlabel("time from EI [min]"); ax.set_ylabel("pitch rate q [deg/s]")
ax.set_title("Pitch rate")

ax = axs[2]
ax.plot(traj["mach"][entry], traj["alpha_deg"][entry], color=S1, linewidth=1.2)
ax.set_xlabel("Mach"); ax.set_ylabel("angle of attack α [deg]")
ax.set_title("AoA vs Mach")
ax.invert_xaxis()

for a in axs.flat:
    style(a)
fig.tight_layout()
fig.savefig(os.path.join(PLOTS, "nominal_attitude.png"), dpi=150)
plt.close(fig)

# ============================ 4. ground track ================================
try:
    from mpl_toolkits.basemap import Basemap
    HAVE_BASEMAP = True
except ImportError:
    HAVE_BASEMAP = False

lon, lat = traj["lon_deg"], traj["lat_deg"]

fig, ax = plt.subplots(figsize=(11, 6.5))
fig.suptitle("Ground track — deorbit to west-coast splashdown", fontweight="bold", color=INK)
if HAVE_BASEMAP:
    m = Basemap(projection="merc", llcrnrlat=-40, urcrnrlat=55,
                llcrnrlon=100, urcrnrlon=-95 + 360, resolution="l", ax=ax)
    m.drawcoastlines(color=MUTED, linewidth=0.6)
    m.fillcontinents(color="#f0efec", lake_color=SURFACE)
    m.drawparallels(np.arange(-40, 61, 20), labels=[1, 0, 0, 0],
                    color=GRID, textcolor=MUTED, fontsize=8)
    m.drawmeridians(np.arange(100, 271, 30), labels=[0, 0, 0, 1],
                    color=GRID, textcolor=MUTED, fontsize=8)

    lon_u = np.where(lon < 0, lon + 360, lon)  # unwrap across the dateline
    xs, ys = m(lon_u, lat)
    pre = traj["t_s"] < t_ei
    m.plot(xs[pre], ys[pre], color=S1, linewidth=2.0, label="orbital phase (>120 km)")
    m.plot(xs[~pre], ys[~pre], color=S2, linewidth=2.4, label="entry phase (<120 km)")
    for n, lab in [("entry_interface", "EI"), ("splashdown", "splashdown")]:
        ex, ey = m(events["lon_deg"][EV[n]] % 360, events["lat_deg"][EV[n]])
        m.plot(ex, ey, "o", ms=7, color=INK, mfc=SURFACE, zorder=6)
        ax.annotate(lab, (ex, ey), textcoords="offset points", xytext=(8, 8),
                    fontsize=9, color=INK)
    tx, ty = m(TARGET[1] % 360, TARGET[0])
    m.plot(tx, ty, "*", ms=14, color=S3, zorder=6, label="target")
    ax.legend(frameon=False, fontsize=9, loc="lower left")
else:
    ax.plot(lon, lat, color=S1)
    ax.set_xlabel("longitude [deg]"); ax.set_ylabel("latitude [deg]")
fig.tight_layout()
fig.savefig(os.path.join(PLOTS, "groundtrack.png"), dpi=150)
plt.close(fig)

# ============================ 5. Monte Carlo footprint =======================
ok = mc["terminated"] == "splashdown"
mlat, mlon = mc["lat_deg"][ok], mc["lon_deg"][ok]
kx = 111.32 * math.cos(math.radians(np.mean(mlat)))  # km per deg lon
ky = 110.57                                          # km per deg lat
dx = (mlon - TARGET[1]) * kx
dy = (mlat - TARGET[0]) * ky

fig, axs = plt.subplots(1, 2, figsize=(13, 5.8),
                        gridspec_kw={"width_ratios": [1.25, 1]})
fig.suptitle(f"Monte Carlo splashdown footprint (n={int(ok.sum())})",
             fontweight="bold", color=INK)

ax = axs[0]
if HAVE_BASEMAP:
    pad = 3.5
    m = Basemap(projection="merc",
                llcrnrlat=TARGET[0] - pad, urcrnrlat=TARGET[0] + pad + 1,
                llcrnrlon=TARGET[1] - pad - 2, urcrnrlon=TARGET[1] + pad + 2,
                resolution="i", ax=ax)
    m.drawcoastlines(color=MUTED, linewidth=0.7)
    m.fillcontinents(color="#f0efec", lake_color=SURFACE)
    m.drawparallels(np.arange(25, 45, 2), labels=[1, 0, 0, 0], color=GRID,
                    textcolor=MUTED, fontsize=8)
    m.drawmeridians(np.arange(-130, -110, 2), labels=[0, 0, 0, 1], color=GRID,
                    textcolor=MUTED, fontsize=8)
    sx, sy = m(mlon, mlat)
    m.scatter(sx, sy, s=14, color=S1, alpha=0.55, edgecolors="none", zorder=5,
              label="MC splashdowns")
    tx, ty = m(*TARGET[::-1])
    m.plot(tx, ty, "*", ms=16, color=S2, zorder=7, label="target")
    ax.legend(frameon=False, fontsize=9, loc="upper left")
else:
    ax.scatter(mlon, mlat, s=14, color=S1, alpha=0.55, edgecolors="none")
    ax.plot(*TARGET[::-1], "*", ms=16, color=S2)
ax.set_title("Geographic scatter")

ax = axs[1]
ax.scatter(dx, dy, s=14, color=S1, alpha=0.55, edgecolors="none", zorder=4)
ax.plot(0, 0, "*", ms=16, color=S2, zorder=6)
ax.annotate("target", (0, 0), textcoords="offset points", xytext=(10, -4),
            fontsize=9, color=INK2)
# 1-sigma and 3-sigma covariance ellipses of the scatter
mx, my = dx.mean(), dy.mean()
cov = np.cov(np.vstack([dx, dy]))
evals, evecs = np.linalg.eigh(cov)
order = evals.argsort()[::-1]
evals, evecs = evals[order], evecs[:, order]
ang = math.degrees(math.atan2(evecs[1, 0], evecs[0, 0]))
for ns, lab in [(1, "1σ"), (3, "3σ")]:
    e = Ellipse((mx, my), 2 * ns * math.sqrt(evals[0]), 2 * ns * math.sqrt(evals[1]),
                angle=ang, fill=False, color=INK2, linewidth=1.2,
                linestyle="-" if ns == 1 else ":")
    ax.add_patch(e)
    ax.annotate(lab, (mx + ns * math.sqrt(evals[0]) * math.cos(math.radians(ang)),
                      my + ns * math.sqrt(evals[0]) * math.sin(math.radians(ang))),
                fontsize=9, color=INK2)
ax.set_xlabel("east of target [km]"); ax.set_ylabel("north of target [km]")
ax.set_title("Dispersion about target")
ax.axis("equal")
style(ax)
fig.tight_layout()
fig.savefig(os.path.join(PLOTS, "montecarlo_footprint.png"), dpi=150)
plt.close(fig)

# ============================ 6. Monte Carlo statistics ======================
fig, axs = plt.subplots(1, 3, figsize=(13, 4.2))
fig.suptitle("Monte Carlo dispersion statistics", fontweight="bold", color=INK)

ax = axs[0]
ax.hist(mc["miss_km"][ok], bins=30, color=S1, edgecolor=SURFACE, linewidth=0.8)
ax.set_xlabel("miss distance from target [km]"); ax.set_ylabel("runs")
ax.set_title("Miss distance")

ax = axs[1]
ax.scatter(mc["mass_kg"][ok], mc["miss_km"][ok], s=14, color=S1, alpha=0.55,
           edgecolors="none")
ax.set_xlabel("dispersed mass [kg]"); ax.set_ylabel("miss distance [km]")
ax.set_title("Miss vs vehicle mass")

ax = axs[2]
ax.scatter(mc["rho_mult"][ok], mc["miss_km"][ok], s=14, color=S1, alpha=0.55,
           edgecolors="none")
ax.set_xlabel("atmosphere density multiplier"); ax.set_ylabel("miss distance [km]")
ax.set_title("Miss vs density dispersion")

for a in axs.flat:
    style(a)
fig.tight_layout()
fig.savefig(os.path.join(PLOTS, "montecarlo_stats.png"), dpi=150)
plt.close(fig)

print("Wrote plots to", PLOTS)
for f in sorted(os.listdir(PLOTS)):
    print("  ", f)
