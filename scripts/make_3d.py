#!/usr/bin/env python3
"""3D visualization of the circumlunar free-return mission.

Reads the CSVs written by scripts/run_moonshot.jl and renders:
  output/plots/moonshot_3d_ascent.png    launch -> parking orbit (ECI)
  output/plots/moonshot_3d_cislunar.png  full Earth-Moon trajectory (ECI)
  output/plots/moonshot_rotating.png     Earth-Moon rotating frame (figure-8)
  output/plots/moonshot_3d_entry.png     entry -> splashdown (ECEF)

Usage: python3 scripts/make_3d.py
"""
import csv
import math
import os

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "output")
PLOTS = os.path.join(OUT, "plots")
os.makedirs(PLOTS, exist_ok=True)

# --- palette (validated dark-mode categorical slots on the space surface) ----
SURFACE = "#1a1a19"
INK = "#ffffff"
INK2 = "#c3c2b7"
MUTED = "#898781"
GRID = "#2c2c2a"
C_PARK = "#3987e5"     # slot 1  parking orbit
C_BURN = "#d95926"     # slot 2  TLI burn
C_OUT = "#199e70"      # slot 3  outbound leg
C_RET = "#c98500"      # slot 4  return leg
C_ENTRY = "#d55181"    # slot 5  entry
EARTH = "#2c4a63"
MOON = "#6b6b66"

RE = 6371.0088         # mean Earth radius [km]
RM = 1737.4            # Moon radius [km]


def read_csv(path):
    with open(path) as f:
        rows = list(csv.reader(f))
    hdr = rows[0]
    data = {h: [] for h in hdr}
    for r in rows[1:]:
        for h, v in zip(hdr, r):
            data[h].append(float(v) if v not in ("", "NaN") else float("nan"))
    return {k: np.array(v) for k, v in data.items()}


def style_ax3d(ax):
    ax.set_facecolor(SURFACE)
    for pane in (ax.xaxis, ax.yaxis, ax.zaxis):
        pane.set_pane_color((0, 0, 0, 0))
        pane.label.set_color(MUTED)
        pane.line.set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=8)
    ax.grid(True)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis._axinfo["grid"].update(color=GRID, linewidth=0.5)


def sphere(ax, cx, cy, cz, r, color, alpha=1.0, n=40, zorder=1):
    u = np.linspace(0, 2 * np.pi, n)
    v = np.linspace(0, np.pi, n // 2)
    x = cx + r * np.outer(np.cos(u), np.sin(v))
    y = cy + r * np.outer(np.sin(u), np.sin(v))
    z = cz + r * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(x, y, z, color=color, alpha=alpha, linewidth=0,
                    antialiased=True, shade=True, zorder=zorder)


def equal_3d(ax, xs, ys, zs, pad=1.05):
    cx, cy, cz = (np.mean([np.min(a), np.max(a)]) for a in (xs, ys, zs))
    r = max(np.max(np.abs(a - c)) for a, c in ((xs, cx), (ys, cy), (zs, cz))) * pad
    ax.set_xlim(cx - r, cx + r)
    ax.set_ylim(cy - r, cy + r)
    ax.set_zlim(cz - r, cz + r)
    try:
        ax.set_box_aspect((1, 1, 1))
    except AttributeError:
        pass


def fig_ax(title, subtitle):
    fig = plt.figure(figsize=(10, 8.5), facecolor=SURFACE)
    ax = fig.add_subplot(111, projection="3d", computed_zorder=False)
    fig.suptitle(title, color=INK, fontsize=14, fontweight="bold", y=0.97)
    ax.set_title(subtitle, color=INK2, fontsize=9, pad=0)
    style_ax3d(ax)
    return fig, ax


# ============================== 1. ascent ====================================
asc = read_csv(os.path.join(OUT, "moonshot_ascent.csv"))
x, y, z = asc["x_m"] / 1e3, asc["y_m"] / 1e3, asc["z_m"] / 1e3

fig, ax = fig_ax("Launch to parking orbit",
                 "3-DOF ascent over rotating Earth — vertical rise, pitch-over, "
                 "gravity turn, closed-loop upper stage")

# Earth patch under the trajectory
lat0 = math.radians(28.5)
lon_c = math.atan2(np.mean(y), np.mean(x))
u = np.linspace(lon_c - 0.30, lon_c + 0.22, 60)
v = np.linspace(lat0 - 0.16, lat0 + 0.16, 40)
U, V = np.meshgrid(u, v)
ax.plot_surface(RE * np.cos(V) * np.cos(U), RE * np.cos(V) * np.sin(U),
                RE * np.sin(V), color=EARTH, alpha=0.7, linewidth=0, zorder=1)

ax.plot(x, y, z, color=C_PARK, lw=1.8, zorder=10)

# events from the combined timeline
ev = {}
with open(os.path.join(OUT, "moonshot_events.csv")) as f:
    for row in csv.DictReader(f):
        if row["phase"] == "ascent":
            ev[row["event"]] = float(row["t_s"])
t = asc["t_s"]
marks = [("liftoff", "liftoff"), ("sep_sable1", "stage 1 sep"),
         ("fairing_jettison", "fairing"), ("seco", "SECO")]
for key, label in marks:
    if key in ev:
        i = int(np.argmin(np.abs(t - ev[key])))
        ax.scatter(x[i], y[i], z[i], color=INK, s=14, zorder=11)
        ax.text(x[i], y[i], z[i] + 60, f" {label}", color=INK2, fontsize=8, zorder=12)

equal_3d(ax, x, y, z, pad=1.12)
ax.set_xlabel("ECI x [km]", fontsize=8)
ax.set_ylabel("ECI y [km]", fontsize=8)
ax.set_zlabel("ECI z [km]", fontsize=8)
ax.view_init(elev=16, azim=-28)
fig.savefig(os.path.join(PLOTS, "moonshot_3d_ascent.png"), dpi=150,
            facecolor=SURFACE, bbox_inches="tight")
plt.close(fig)

# ============================ 2. cislunar ====================================
cis = read_csv(os.path.join(OUT, "moonshot_cislunar.csv"))
X, Y, Z = cis["x_m"] / 1e6, cis["y_m"] / 1e6, cis["z_m"] / 1e6   # in 1000 km
MX, MY, MZ = cis["moon_x_m"] / 1e6, cis["moon_y_m"] / 1e6, cis["moon_z_m"] / 1e6
ph = cis["phase"].astype(int)
T = cis["t_s"]

# the designed mission is exactly coplanar (checked: out-of-plane < 1 km over
# 20 days), so the honest picture is the mission plane itself, top-down —
# a 3D rendering of a planar trajectory only adds foreshortening
b1 = np.array([MX[0], MY[0], MZ[0]])
b1 = b1 / np.linalg.norm(b1)
k = len(MX) // 3
nrm = np.cross([MX[0], MY[0], MZ[0]], [MX[k], MY[k], MZ[k]])
nrm = nrm / np.linalg.norm(nrm)
b2 = np.cross(nrm, b1)
R = np.stack([b1, b2, nrm])              # world -> plane frame

traj = R @ np.stack([X, Y, Z])
Xp, Yp = traj[0], traj[1]
moon = R @ np.stack([MX, MY, MZ])
MXp, MYp = moon[0], moon[1]

fig = plt.figure(figsize=(10, 9), facecolor=SURFACE)
ax = fig.add_subplot(111)
ax.set_facecolor(SURFACE)
fig.suptitle("Circumlunar free return — inertial mission plane", color=INK,
             fontsize=14, fontweight="bold", y=0.96)
ax.set_title("launch, TLI, 2000 km lunar flyby and coast home; "
             "Moon shown at the epochs that matter", color=INK2, fontsize=9)

th = np.linspace(0, 2 * np.pi, 300)
rm = np.sqrt(MXp**2 + MYp**2).mean()
ax.plot(rm * np.cos(th), rm * np.sin(th), color=GRID, lw=0.9, zorder=2)
ax.add_patch(plt.Circle((0, 0), RE / 1e3 * 3, color=EARTH, zorder=6))
ax.text(12, -32, "Earth (3×)", color=INK2, fontsize=9)

segs = [(0, C_PARK, "parking orbit"), (1, C_BURN, "TLI burn"),
        (2, C_OUT, "outbound"), (3, C_RET, "return")]
for p, c, label in segs:
    m = ph == p
    if m.any():
        ax.plot(Xp[m], Yp[m], color=c, lw=1.8 if p != 1 else 3.2, zorder=10)

events = {}
with open(os.path.join(OUT, "moonshot_events.csv")) as f:
    for row in csv.DictReader(f):
        if row["phase"] == "cislunar":
            events[row["event"]] = float(row["t_s"])
moon_marks = [("tli_ignition", "Moon at TLI", (10, -22)),
              ("perilune", "Moon at flyby", (8, 14)),
              ("entry_handoff", "Moon at entry", (8, 12))]
for key, label, off in moon_marks:
    i = int(np.argmin(np.abs(T - events[key])))
    ax.add_patch(plt.Circle((MXp[i], MYp[i]), RM / 1e3 * 3, color=MOON, zorder=8))
    ax.annotate(label, (MXp[i], MYp[i]), textcoords="offset points",
                xytext=off, color=INK2, fontsize=8, zorder=12)

ip = int(np.argmin(np.abs(T - events["perilune"])))
jo = int(np.argmin(np.abs(T - 1.8 * 86400)))
jr = int(np.argmin(np.abs(T - 11.0 * 86400)))
ax.annotate("outbound (3.3 d)", (Xp[jo], Yp[jo]), textcoords="offset points",
            xytext=(-8, 10), color=C_OUT, fontsize=9, ha="right", zorder=12)
ax.annotate("return (16 d)", (Xp[jr], Yp[jr]), textcoords="offset points",
            xytext=(6, -14), color=C_RET, fontsize=9, zorder=12)

ax.set_aspect("equal")
ax.set_xlabel("in-plane x [1000 km]", color=MUTED, fontsize=9)
ax.set_ylabel("in-plane y [1000 km]", color=MUTED, fontsize=9)
ax.tick_params(colors=MUTED, labelsize=8)
for s in ax.spines.values():
    s.set_color(GRID)
ax.grid(color=GRID, lw=0.5)
ax.legend([plt.Line2D([0], [0], color=c, lw=2.4) for _, c, _ in segs],
          [s[2] for s in segs], loc="upper left", facecolor=SURFACE,
          edgecolor=GRID, labelcolor=INK2, fontsize=8)
fig.savefig(os.path.join(PLOTS, "moonshot_3d_cislunar.png"), dpi=150,
            facecolor=SURFACE, bbox_inches="tight")
plt.close(fig)

# ======================= 3. rotating frame (figure-8) ========================
fig = plt.figure(figsize=(10, 6.4), facecolor=SURFACE)
ax = fig.add_subplot(111)
ax.set_facecolor(SURFACE)
fig.suptitle("Free return in the Earth-Moon rotating frame", color=INK,
             fontsize=14, fontweight="bold", y=0.97)
ax.set_title("x axis pinned to the Earth-Moon line — the classic figure-8",
             color=INK2, fontsize=9)

# rotating-frame coordinates: x along Earth->Moon, y in-plane
mvec = np.stack([MX, MY, MZ], axis=1)
rvec = np.stack([X, Y, Z], axis=1)
mhat = mvec / np.linalg.norm(mvec, axis=1, keepdims=True)
zhat = np.cross(mvec[0], mvec[len(mvec) // 3])
zhat = zhat / np.linalg.norm(zhat)
yhat = np.cross(zhat, mhat)
xi = np.einsum("ij,ij->i", rvec, mhat)
eta = np.einsum("ij,ij->i", rvec, yhat)

for p, c, label in segs:
    m = ph == p
    if m.any():
        ax.plot(xi[m], eta[m], color=c, lw=1.8 if p != 1 else 3.0, zorder=10)

earth = plt.Circle((0, 0), RE / 1e3, color=EARTH, zorder=5)
moon = plt.Circle((rm, 0), RM / 1e3 * 3, color=MOON, zorder=5)
ax.add_patch(earth)
ax.add_patch(moon)
ax.text(6, -26, "Earth", color=INK2, fontsize=9)
ax.text(rm - 9, -32, "Moon (3×)", color=INK2, fontsize=9)
ax.annotate("perilune", (xi[ip], eta[ip]), textcoords="offset points",
            xytext=(10, 8), color=C_OUT, fontsize=8)

ax.set_aspect("equal")
ax.set_xlabel("distance along Earth-Moon line [1000 km]", color=MUTED, fontsize=9)
ax.set_ylabel("in-plane offset [1000 km]", color=MUTED, fontsize=9)
ax.tick_params(colors=MUTED, labelsize=8)
for s in ax.spines.values():
    s.set_color(GRID)
ax.grid(color=GRID, lw=0.5)
leg = ax.legend([plt.Line2D([0], [0], color=c, lw=2.4) for _, c, _ in segs],
                [s[2] for s in segs], loc="upper left", facecolor=SURFACE,
                edgecolor=GRID, labelcolor=INK2, fontsize=8)
fig.savefig(os.path.join(PLOTS, "moonshot_rotating.png"), dpi=150,
            facecolor=SURFACE, bbox_inches="tight")
plt.close(fig)

# ============================== 4. entry =====================================
ent = read_csv(os.path.join(OUT, "moonshot_entry.csv"))
lat = np.radians(ent["lat_deg"])
lon = np.radians(ent["lon_deg"])
h = ent["alt_m"] / 1e3
r = RE + h
ex = r * np.cos(lat) * np.cos(lon)
ey = r * np.cos(lat) * np.sin(lon)
ez = r * np.sin(lat)

fig, ax = fig_ax("Lunar-return entry — 10.6 km/s at entry interface",
                 "4-DOF ballistic entry, drogue + main parachutes, Pacific splashdown")

lon_c = math.atan2(ey[len(ey) // 2], ex[len(ex) // 2])
lat_c = math.asin(ez[len(ez) // 2] / r[len(r) // 2])
u = np.linspace(lon_c - 0.5, lon_c + 0.35, 60)
v = np.linspace(lat_c - 0.3, lat_c + 0.3, 40)
U, V = np.meshgrid(u, v)
ax.plot_surface(RE * np.cos(V) * np.cos(U), RE * np.cos(V) * np.sin(U),
                RE * np.sin(V), color=EARTH, alpha=0.85, linewidth=0, zorder=1)

# ground track shadow on the surface
gx = RE * np.cos(lat) * np.cos(lon)
gy = RE * np.cos(lat) * np.sin(lon)
gz = RE * np.sin(lat)
ax.plot(gx, gy, gz, color=MUTED, lw=1.0, ls=":", zorder=5)
ax.plot(ex, ey, ez, color=C_ENTRY, lw=1.8, zorder=10)

evn = []
with open(os.path.join(OUT, "moonshot_events.csv")) as f:
    for row in csv.DictReader(f):
        if row["phase"] == "entry":
            evn.append((row["event"], float(row["t_s"])))
tt = ent["t_s"]
# drogue/main deploys are sub-pixel at this scale; label the ends only
lbl = {"entry_interface": "EI 120 km", "splashdown": "splashdown"}
for name, te in evn:
    if name in lbl:
        i = int(np.argmin(np.abs(tt - te)))
        ax.scatter(ex[i], ey[i], ez[i], color=INK, s=14, zorder=11)
        ax.text(ex[i], ey[i], ez[i] + 25, f" {lbl[name]}", color=INK2,
                fontsize=8, zorder=12)

equal_3d(ax, ex, ey, ez, pad=1.25)
ax.set_xlabel("ECEF x [km]", fontsize=8)
ax.set_ylabel("ECEF y [km]", fontsize=8)
ax.set_zlabel("ECEF z [km]", fontsize=8)
ax.view_init(elev=24, azim=-130)
fig.savefig(os.path.join(PLOTS, "moonshot_3d_entry.png"), dpi=150,
            facecolor=SURFACE, bbox_inches="tight")
plt.close(fig)

print("wrote 4 figures to output/plots/")
