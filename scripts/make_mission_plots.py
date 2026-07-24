#!/usr/bin/env python3
"""Render the full circumlunar mission-analysis plot suite.

Usage:  python3 scripts/make_mission_plots.py
Reads   output/moonshot_{ascent,cislunar,entry,events}.csv
        output/tcm_montecarlo.csv            (optional)
Writes  output/plots/mission_*.png, ascent_*.png, entry_*.png, tcm_*.png

Companion to scripts/make_plots.py (reentry-only plots) and
scripts/make_3d.py (3D trajectory views). Time axes that span wildly
different scales (a 8-minute ascent, a 6-day cruise, a 15-minute entry)
use broken x-axes so every phase stays readable.
"""
import csv
import math
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

# --- palette (validated reference palette, light mode; see make_plots.py) ----
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
BASE = "#c3c2b7"
# phase identity colors (fixed assignment, used everywhere in this suite)
C_ASC = "#7aa3d6"      # ascent
C_PARK = "#2a78d6"     # parking orbit
C_BURN = "#eb6834"     # TLI burn
C_OUT = "#1baf7a"      # outbound coast
C_RET = "#9c7400"      # return coast
C_ENTRY = "#cf3e78"    # atmospheric entry

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
            cols[k] = np.array([float(r[k]) if r[k] not in ("", "NaN") else np.nan
                                for r in rows])
        except ValueError:
            cols[k] = np.array([r[k] for r in rows])
    return cols


asc = read_csv(os.path.join(OUT, "moonshot_ascent.csv"))
cis = read_csv(os.path.join(OUT, "moonshot_cislunar.csv"))
ent = read_csv(os.path.join(OUT, "moonshot_entry.csv"))
ev = read_csv(os.path.join(OUT, "moonshot_events.csv"))

EV = {}
for i, name in enumerate(ev["event"]):
    EV.setdefault(name, i)
t_tli = ev["t_s"][EV["tli_ignition"]]
t_tlico = ev["t_s"][EV["tli_cutoff"]] if "tli_cutoff" in EV else t_tli + 190
t_peri = ev["t_s"][EV["perilune"]]
t_ei = ev["t_s"][EV["entry_interface"]]
t_splash = ev["t_s"][EV["splashdown"]]
t_seco = ev["t_s"][EV["seco"]] if "seco" in EV else asc["t_s"][-1]

# derived cislunar series
c_t = cis["t_s"]
c_r = np.hypot(np.hypot(cis["x_m"], cis["y_m"]), cis["z_m"])
c_v = np.hypot(np.hypot(cis["vx_ms"], cis["vy_ms"]), cis["vz_ms"])
c_eps = 0.5 * c_v**2 - MU_E / c_r                       # two-body energy [J/kg]
c_h = np.linalg.norm(np.cross(
    np.stack([cis["x_m"], cis["y_m"], cis["z_m"]], 1),
    np.stack([cis["vx_ms"], cis["vy_ms"], cis["vz_ms"]], 1)), axis=1)
phase = cis["phase"].astype(int)

a_r = np.hypot(np.hypot(asc["x_m"], asc["y_m"]), asc["z_m"])
a_eps = 0.5 * asc["v_inertial_ms"]**2 - MU_E / a_r
e_r = RE + ent["alt_m"]
e_eps = 0.5 * ent["v_inertial_ms"]**2 - MU_E / e_r

PHASE_STYLE = {0: (C_PARK, "parking"), 1: (C_BURN, "TLI burn"),
               2: (C_OUT, "outbound"), 3: (C_RET, "return")}


# ------------------------------------------------------------ broken axes --
def broken_row(axs, drop_ylabels=True):
    """Style a row of axes as one broken axis (share y, slash break marks)."""
    d = 0.5
    kw = dict(marker=[(-1, -d), (1, d)], markersize=10, linestyle="none",
              color=BASE, mec=BASE, mew=1.2, clip_on=False)
    for k, ax in enumerate(axs):
        if k > 0:
            ax.spines["left"].set_visible(False)
            ax.tick_params(left=False, labelleft=True and not drop_ylabels)
            if drop_ylabels:
                ax.tick_params(labelleft=False)
            ax.plot([0, 0], [0, 1], transform=ax.transAxes, **kw)
        if k < len(axs) - 1:
            ax.spines["right"].set_visible(False)
            ax.plot([1, 1], [0, 1], transform=ax.transAxes, **kw)


def cruise_segments(ax, xdays, ys, lw=2.0):
    """Plot the cislunar log split by phase so identity colors persist."""
    for ph in (0, 1, 2, 3):
        m = phase == ph
        if not m.any():
            continue
        # break the line where the mask is discontiguous
        idx = np.where(m)[0]
        splits = np.where(np.diff(idx) > 1)[0]
        for chunk in np.split(idx, splits + 1):
            ax.plot(xdays[chunk], ys[chunk], color=PHASE_STYLE[ph][0],
                    linewidth=lw if ph != 1 else lw + 1.2)


def seg_label(ax, x, y, text, color, dy=8):
    ax.annotate(text, (x, y), textcoords="offset points", xytext=(0, dy),
                fontsize=9, color=color, ha="center")


def ev_dot(ax, x, y, label, off=(8, 6), color=INK):
    ax.plot(x, y, "o", ms=6, color=color, mfc=SURFACE, zorder=6)
    ax.annotate(label, (x, y), textcoords="offset points", xytext=off,
                fontsize=9, color=INK2)


# =============== 1. mission overview — broken time axis ======================
fig, axs = plt.subplots(2, 3, figsize=(13, 8), sharey="row",
                        gridspec_kw={"width_ratios": [1.0, 1.9, 1.0],
                                     "wspace": 0.06, "hspace": 0.3})
fig.suptitle("Circumlunar mission overview — ascent · cruise · entry "
             "(note the broken time axis)", fontweight="bold", color=INK)

ta = asc["t_s"]
td = c_t / 86400.0
te = ent["t_s"] - t_ei

# --- row 1: geodetic altitude (log) --
ax = axs[0, 0]
ax.plot(ta, np.maximum(asc["alt_m"], 100) / 1e3, color=C_ASC)
ax.set_yscale("log")
ax.set_ylim(0.5, 1.2e6)
ax.set_ylabel("geodetic altitude [km] (log)")
ax.set_xlabel("ascent time [s]")
ax.set_title("ascent", color=C_ASC)
ev_dot(ax, t_seco, asc["alt_m"][-1] / 1e3, "insertion", (-52, -4))

ax = axs[0, 1]
cruise_segments(ax, td, cis["alt_m"] / 1e3)
ax.set_xlabel("mission time [days]")
ax.set_title("cruise", color=C_OUT)
i_p = np.argmin(np.abs(c_t - t_peri))
ev_dot(ax, t_peri / 86400, cis["alt_m"][i_p] / 1e3, "perilune", (0, 8))
i_b = np.argmin(np.abs(c_t - t_tli))
ev_dot(ax, t_tli / 86400, cis["alt_m"][i_b] / 1e3, "TLI", (6, 6))
seg_label(ax, td[phase == 2].mean(), 3.6e5, "outbound", C_OUT, 0)
seg_label(ax, td[phase == 3].mean(), 1.2e5, "return", C_RET, 0)

ax = axs[0, 2]
ax.plot(te, np.maximum(ent["alt_m"], 100) / 1e3, color=C_ENTRY)
ax.set_xlabel("entry time from EI [s]")
ax.set_title("entry", color=C_ENTRY)
for n, lab in [("deploy_drogue", "drogue"), ("deploy_main", "main"),
               ("splashdown", "splash")]:
    if n in EV:
        ev_dot(ax, ev["t_s"][EV[n]] - t_ei, max(ev["alt_m"][EV[n]], 100) / 1e3,
               lab, (6, 4))
broken_row(axs[0])

# --- row 2: inertial speed --
ax = axs[1, 0]
ax.plot(ta, asc["v_inertial_ms"] / 1e3, color=C_ASC)
ax.set_ylabel("inertial speed [km/s]")
ax.set_xlabel("ascent time [s]")
for n, lab, off in [("sep_sable1", "staging", (4, -12)), ("seco", "SECO", (-40, -2))]:
    if n in EV:
        i = EV[n]
        ev_dot(ax, ev["t_s"][i], ev["v_ms"][i] / 1e3 if np.isfinite(ev["v_ms"][i])
               else 7.4, lab, off)

ax = axs[1, 1]
cruise_segments(ax, td, c_v / 1e3)
ax.set_xlabel("mission time [days]")
ev_dot(ax, t_peri / 86400, c_v[i_p] / 1e3, "perilune", (0, 8))

ax = axs[1, 2]
ax.plot(te, ent["v_inertial_ms"] / 1e3, color=C_ENTRY)
ax.set_xlabel("entry time from EI [s]")
broken_row(axs[1])

fig.savefig(os.path.join(PLOTS, "mission_overview.png"), dpi=150,
            bbox_inches="tight")
plt.close(fig)

# =============== 2. specific orbital energy — broken time axis ===============
fig, axs = plt.subplots(1, 3, figsize=(13, 4.8), sharey=True,
                        gridspec_kw={"width_ratios": [1.0, 1.9, 1.0],
                                     "wspace": 0.06})
fig.suptitle("Specific two-body orbital energy  v²/2 − μ/r   "
             "(TLI raises it; the lunar flyby and tides perturb it; "
             "drag dumps it)", fontweight="bold", color=INK)

axs[0].plot(ta, a_eps / 1e6, color=C_ASC)
axs[0].set_ylabel("ε [MJ/kg]")
axs[0].set_xlabel("ascent time [s]")
axs[0].set_title("ascent", color=C_ASC)

cruise_segments(axs[1], td, c_eps / 1e6)
axs[1].set_xlabel("mission time [days]")
axs[1].set_title("cruise", color=C_OUT)
axs[1].axhline(0, color=BASE, linewidth=1.0, linestyle=":")
axs[1].annotate("escape energy", (td[len(td) // 2], 0.35), fontsize=9,
                color=MUTED, ha="center")
ev_dot(axs[1], t_peri / 86400, c_eps[i_p] / 1e6, "lunar flyby", (6, -14))
ev_dot(axs[1], t_tli / 86400, c_eps[i_b] / 1e6, "TLI", (8, -4))

axs[2].plot(te, e_eps / 1e6, color=C_ENTRY)
axs[2].set_xlabel("entry time from EI [s]")
axs[2].set_title("entry", color=C_ENTRY)
broken_row(axs)
fig.savefig(os.path.join(PLOTS, "mission_energy.png"), dpi=150,
            bbox_inches="tight")
plt.close(fig)

# cruise-only zoom: the lunar tides' work on the two-body energy is a small
# signal, invisible at full mission scale
fig, ax = plt.subplots(figsize=(10, 4.6))
fig.suptitle("Cruise energy detail — what the Moon's gravity does",
             fontweight="bold", color=INK)
post = c_t > t_tlico + 60
cruise_segments(ax, td, np.where(post, c_eps, np.nan) / 1e6)
ax.set_xlabel("mission time [days]")
ax.set_ylabel("ε [MJ/kg]")
ax.axhline(0, color=BASE, linewidth=1.0, linestyle=":")
ax.annotate("escape energy", (td[post].mean(), 0.02), fontsize=9, color=MUTED)
ev_dot(ax, t_peri / 86400, c_eps[i_p] / 1e6, "perilune", (6, -14))
lo = np.nanmin(np.where(post, c_eps, np.nan)) / 1e6
ax.set_ylim(lo * 1.35, 0.12)
fig.tight_layout()
fig.savefig(os.path.join(PLOTS, "mission_energy_cruise.png"), dpi=150,
            bbox_inches="tight")
plt.close(fig)

# =============== 3. angular momentum & 4. distances ==========================
fig, axs = plt.subplots(1, 2, figsize=(13, 4.6))
fig.suptitle("Cruise invariants — what the Moon does to the orbit",
             fontweight="bold", color=INK)

ax = axs[0]
cruise_segments(ax, td, c_h / 1e9)
ax.set_xlabel("mission time [days]")
ax.set_ylabel("|r × v| [km²/s ×10⁶]")
ax.set_title("Specific angular momentum (the flyby's torque)")
ev_dot(ax, t_peri / 86400, c_h[i_p] / 1e9, "perilune", (6, 6))

ax = axs[1]
ax.plot(td, c_r / 1e6, color=C_PARK)
ax.plot(td, cis["d_moon_m"] / 1e6, color=C_OUT)
ax.set_xlabel("mission time [days]")
ax.set_ylabel("distance [1000 km]")
ax.set_title("Distance to Earth and Moon")
ax.annotate("to Earth", (td[-1], c_r[-1] / 1e6), textcoords="offset points",
            xytext=(-52, 8), fontsize=9, color=C_PARK)
ax.annotate("to Moon", (td[-1], cis["d_moon_m"][-1] / 1e6),
            textcoords="offset points", xytext=(-52, -14), fontsize=9, color=C_OUT)
ev_dot(ax, t_peri / 86400, cis["d_moon_m"][i_p] / 1e6, "perilune 2000 km", (0, 10))
fig.tight_layout()
fig.savefig(os.path.join(PLOTS, "mission_cruise.png"), dpi=150,
            bbox_inches="tight")
plt.close(fig)

# =============== 5. ascent profile ==========================================
fig, axs = plt.subplots(2, 3, figsize=(13, 7.6))
fig.suptitle("Ascent — pad to parking-orbit insertion", fontweight="bold",
             color=INK)
panels = [
    ("alt_m", 1e3, "altitude [km]"),
    ("v_rel_ms", 1e3, "relative speed [km/s]"),
    ("gamma_deg", 1, "flight-path angle [deg]"),
    ("qbar_pa", 1e3, "dynamic pressure [kPa]"),
    ("mach", 1, "Mach"),
    ("mass_kg", 1e3, "stack mass [t]"),
]
ascent_events = [("pitchover", "kick"), ("sep_sable1", "stg1"),
                 ("fairing_jettison", "fairing"), ("seco", "SECO")]
for k, (col, div, label) in enumerate(panels):
    ax = axs[k // 3, k % 3]
    ax.plot(ta, asc[col] / div, color=C_ASC)
    ax.set_ylabel(label)
    ax.set_xlabel("time [s]")
    for n, lab in ascent_events:
        if n in EV:
            ax.axvline(ev["t_s"][EV[n]], color=BASE, linewidth=1.0,
                       linestyle=":")
            if k == 0:
                ax.annotate(lab, (ev["t_s"][EV[n]], ax.get_ylim()[1] * 0.9),
                            fontsize=8, color=MUTED, rotation=90, va="top")
fig.tight_layout()
fig.savefig(os.path.join(PLOTS, "ascent_profile.png"), dpi=150,
            bbox_inches="tight")
plt.close(fig)

# downrange view
fig, ax = plt.subplots(figsize=(9, 4.6))
fig.suptitle("Ascent trajectory — altitude vs downrange", fontweight="bold",
             color=INK)
ax.plot(asc["downrange_m"] / 1e3, asc["alt_m"] / 1e3, color=C_ASC)
ax.set_xlabel("downrange [km]")
ax.set_ylabel("altitude [km]")
ax.set_aspect("equal", adjustable="box")
fig.tight_layout()
fig.savefig(os.path.join(PLOTS, "ascent_trajectory.png"), dpi=150,
            bbox_inches="tight")
plt.close(fig)

# =============== 6. entry profile ===========================================
fig, axs = plt.subplots(2, 3, figsize=(13, 7.6))
fig.suptitle("Lunar-return entry — 10.6 km/s ballistic", fontweight="bold",
             color=INK)
entry_events = [("deploy_drogue", "drogue"), ("deploy_main", "main"),
                ("splashdown", "splash")]


def entry_vlines(ax):
    for n, _ in entry_events:
        if n in EV:
            ax.axvline(ev["t_s"][EV[n]] - t_ei, color=BASE, linewidth=1.0,
                       linestyle=":")


ax = axs[0, 0]
ax.plot(te, ent["alt_m"] / 1e3, color=C_ENTRY)
ax.set_ylabel("altitude [km]"); ax.set_xlabel("time from EI [s]")
entry_vlines(ax)
for n, lab in entry_events:
    if n in EV:
        ax.annotate(lab, (ev["t_s"][EV[n]] - t_ei, ent["alt_m"].max() / 1e3 * 0.9),
                    fontsize=8, color=MUTED, rotation=90, va="top")

for k, (col, div, label) in enumerate([
        ("v_rel_ms", 1e3, "relative speed [km/s]"),
        ("gload", 1, "deceleration [g]"),
        ("qbar_pa", 1e3, "dynamic pressure [kPa]"),
        ("mach", 1, "Mach"),
]):
    ax = axs[(k + 1) // 3, (k + 1) % 3]
    ax.plot(te, ent[col] / div, color=C_ENTRY)
    ax.set_ylabel(label); ax.set_xlabel("time from EI [s]")
    entry_vlines(ax)

ax = axs[1, 2]
ax.plot(te, ent["qdot_conv_wcm2"], color=C_ENTRY)
ax.plot(te, ent["qdot_rad_wcm2"], color=C_RET)
ax.set_ylabel("stagnation heat flux [W/cm²]"); ax.set_xlabel("time from EI [s]")
ipk = int(np.argmax(ent["qdot_conv_wcm2"]))
ax.annotate("convective", (te[ipk], ent["qdot_conv_wcm2"][ipk]),
            textcoords="offset points", xytext=(10, 2), fontsize=9, color=C_ENTRY)
irk = int(np.argmax(ent["qdot_rad_wcm2"]))
ax.annotate("radiative", (te[irk], ent["qdot_rad_wcm2"][irk]),
            textcoords="offset points", xytext=(10, -12), fontsize=9, color=C_RET)
entry_vlines(ax)
fig.tight_layout()
fig.savefig(os.path.join(PLOTS, "entry_profile.png"), dpi=150,
            bbox_inches="tight")
plt.close(fig)

# corridor: altitude vs velocity with the story points
fig, ax = plt.subplots(figsize=(9, 5.2))
fig.suptitle("Entry corridor — altitude vs relative velocity",
             fontweight="bold", color=INK)
ax.plot(ent["v_rel_ms"] / 1e3, ent["alt_m"] / 1e3, color=C_ENTRY)
ax.set_xlabel("relative velocity [km/s]"); ax.set_ylabel("altitude [km]")
ipg = int(np.argmax(ent["gload"]))
ipq = int(np.argmax(ent["qdot_conv_wcm2"] + ent["qdot_rad_wcm2"]))
ev_dot(ax, ent["v_rel_ms"][0] / 1e3, ent["alt_m"][0] / 1e3, "EI 140 km", (8, 4))
ev_dot(ax, ent["v_rel_ms"][ipq] / 1e3, ent["alt_m"][ipq] / 1e3,
       f"peak heating {(ent['qdot_conv_wcm2'] + ent['qdot_rad_wcm2'])[ipq]:.0f} W/cm²",
       (10, 4))
ev_dot(ax, ent["v_rel_ms"][ipg] / 1e3, ent["alt_m"][ipg] / 1e3,
       f"peak load {ent['gload'][ipg]:.1f} g", (10, -12))
for n, lab in entry_events:
    if n in EV:
        i = int(np.argmin(np.abs(ent["t_s"] - ev["t_s"][EV[n]])))
        ev_dot(ax, ent["v_rel_ms"][i] / 1e3, ent["alt_m"][i] / 1e3, lab, (10, 0))
fig.tight_layout()
fig.savefig(os.path.join(PLOTS, "entry_corridor.png"), dpi=150,
            bbox_inches="tight")
plt.close(fig)

# aerothermal detail
fig, axs = plt.subplots(1, 3, figsize=(13, 4.4))
fig.suptitle("Entry aerothermal environment", fontweight="bold", color=INK)
ax = axs[0]
ax.plot(te, ent["qdot_conv_wcm2"], color=C_ENTRY)
ax.plot(te, ent["qdot_rad_wcm2"], color=C_RET)
ax.set_ylabel("heat flux [W/cm²]"); ax.set_xlabel("time from EI [s]")
ax.annotate("convective", (te[ipk], ent["qdot_conv_wcm2"][ipk]),
            textcoords="offset points", xytext=(10, 2), fontsize=9, color=C_ENTRY)
ax.annotate("radiative", (te[irk], ent["qdot_rad_wcm2"][irk]),
            textcoords="offset points", xytext=(10, -12), fontsize=9, color=C_RET)
axs[1].plot(te, ent["heat_load_jcm2"] / 100, color=C_ENTRY)   # J/cm² -> MJ/m²
axs[1].set_ylabel("integrated heat load [MJ/m²]")
axs[1].set_xlabel("time from EI [s]")
axs[2].plot(te, ent["t_wall_k"], color=C_ENTRY)
axs[2].set_ylabel("radiative-equilibrium wall T [K]")
axs[2].set_xlabel("time from EI [s]")
fig.tight_layout()
fig.savefig(os.path.join(PLOTS, "entry_heating.png"), dpi=150,
            bbox_inches="tight")
plt.close(fig)

# attitude
fig, axs = plt.subplots(1, 2, figsize=(11, 4.2))
fig.suptitle("Entry attitude — the 4th degree of freedom", fontweight="bold",
             color=INK)
axs[0].plot(te, ent["alpha_deg"], color=C_ENTRY)
axs[0].set_ylabel("angle of attack [deg]"); axs[0].set_xlabel("time from EI [s]")
axs[1].plot(te, ent["pitch_rate_dps"], color=C_ENTRY)
axs[1].set_ylabel("pitch rate [deg/s]"); axs[1].set_xlabel("time from EI [s]")
for ax in axs:
    entry_vlines(ax)
fig.tight_layout()
fig.savefig(os.path.join(PLOTS, "entry_attitude.png"), dpi=150,
            bbox_inches="tight")
plt.close(fig)

# =============== 7. ground tracks ===========================================
try:
    from mpl_toolkits.basemap import Basemap
    HAVE_BASEMAP = True
except ImportError:
    HAVE_BASEMAP = False


def land_mask():
    """256x128 equirect land mask embedded in the panel page (1 bit/px)."""
    import base64
    import re
    try:
        html = open(os.path.join(ROOT, "scripts", "panel_page.html"), encoding="utf-8").read()
        m = re.search(r"LAND_B64 = '([^']+)'", html)
        raw = base64.b64decode(m.group(1))
        bits = np.unpackbits(np.frombuffer(raw, dtype=np.uint8))
        return bits[:128 * 256].reshape(128, 256)
    except Exception:
        return None


fig, ax = plt.subplots(figsize=(11, 5.6))
fig.suptitle("Mission ground track — launch and entry phases",
             fontweight="bold", color=INK)
if HAVE_BASEMAP:
    m = Basemap(projection="cyl", llcrnrlat=-65, urcrnrlat=65,
                llcrnrlon=-180, urcrnrlon=180, resolution="c", ax=ax)
    m.drawcoastlines(color=MUTED, linewidth=0.6)
    m.fillcontinents(color="#f0efec", lake_color=SURFACE)
    to_map = lambda lo, la: m(lo, la)
else:
    from matplotlib.colors import ListedColormap
    lm = land_mask()
    if lm is not None:
        ax.imshow(lm, extent=(-180, 180, -90, 90), origin="upper",
                  cmap=ListedColormap([SURFACE, "#e9e7dc"]), aspect="auto",
                  zorder=0, interpolation="nearest")
    ax.set_xlim(-180, 180); ax.set_ylim(-65, 65)
    ax.set_xlabel("longitude [deg]"); ax.set_ylabel("latitude [deg]")
    to_map = lambda lo, la: (lo, la)

ax.plot(*to_map(asc["lon_deg"], asc["lat_deg"]), color=C_ASC, linewidth=2.2)
lon_e, lat_e = ent["lon_deg"].copy(), ent["lat_deg"]
jump = np.abs(np.diff(lon_e)) > 180
lon_plot = lon_e.copy()
lon_plot[1:][jump] = np.nan            # break the line across the dateline
ax.plot(*to_map(lon_plot, lat_e), color=C_ENTRY, linewidth=2.2)
x0, y0 = to_map(asc["lon_deg"][0], asc["lat_deg"][0])
ax.plot(x0, y0, "^", ms=9, color=C_ASC, zorder=6)
ax.annotate("launch", (x0, y0), textcoords="offset points", xytext=(8, 6),
            fontsize=9, color=INK2)
xs, ys = to_map(lon_e[-1], lat_e[-1])
ax.plot(xs, ys, "*", ms=14, color=C_ENTRY, zorder=6)
ax.annotate("splashdown", (xs, ys), textcoords="offset points", xytext=(8, 6),
            fontsize=9, color=INK2)
fig.tight_layout()
fig.savefig(os.path.join(PLOTS, "mission_groundtrack.png"), dpi=150,
            bbox_inches="tight")
plt.close(fig)

# =============== 8. TCM Monte Carlo (optional) ==============================
tcm_path = os.path.join(OUT, "tcm_montecarlo.csv")
if os.path.exists(tcm_path):
    tcm = read_csv(tcm_path)
    ok = tcm["corridor_ok"] == 1
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.8))
    fig.suptitle(f"TLI execution errors + mid-course correction "
                 f"(n={len(ok)}, corridor {int(ok.sum())}/{len(ok)})",
                 fontweight="bold", color=INK)
    ax = axs[0]
    ax.hist(tcm["tcm_dv_ms"][np.isfinite(tcm["tcm_dv_ms"])], bins=24,
            color=C_PARK, edgecolor=SURFACE, linewidth=1.5)
    ax.set_xlabel("TCM Δv [m/s]"); ax.set_ylabel("runs")
    ax.set_title("Correction budget")
    ax = axs[1]
    err = 100 * tcm["mag_err"]
    ax.scatter(err[ok], tcm["tcm_dv_ms"][ok], s=18, color=C_PARK,
               edgecolors="none", label="corridor ok")
    if (~ok).any():
        ax.scatter(err[~ok], tcm["tcm_dv_ms"][~ok], s=26, marker="x",
                   color=C_BURN, label="missed corridor")
    ax.set_xlabel("TLI magnitude error [%]"); ax.set_ylabel("TCM Δv [m/s]")
    ax.set_title("Δv vs injection error")
    ax.legend(frameon=False, fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(PLOTS, "tcm_montecarlo.png"), dpi=150,
                bbox_inches="tight")
    plt.close(fig)

names = sorted(f for f in os.listdir(PLOTS) if f.endswith(".png"))
print(f"wrote {len(names)} plots to output/plots/:")
for n in names:
    print("  ", n)
