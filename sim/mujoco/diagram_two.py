# Draws sim/screenshots/two-link-diagram.png: the two-link scenario (sim graph --scenario sim/scenario-2link.yaml) with the per-tick data flow
# of sim/peers/plant2.py and world.py.
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
fig, ax = plt.subplots(figsize=(12, 7.4), dpi=110); ax.set_xlim(0, 12); ax.set_ylim(0, 7.4); ax.axis("off")
def box(x, y, w, h, text, fc, fs=9, bold=False, ls="-", z=2):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.08", fc=fc, ec="#333", lw=1.2, ls=ls, zorder=z))
    if text: ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs, weight="bold" if bold else "normal", zorder=z + 1)
def arrow(a, b, text="", both=False, off=(0, 0.13), ha="center"):
    ax.annotate("", xy=b, xytext=a, arrowprops=dict(arrowstyle="<->" if both else "->", color="#222", lw=1.3), zorder=5)
    if text: ax.text((a[0] + b[0]) / 2 + off[0], (a[1] + b[1]) / 2 + off[1], text, ha=ha, fontsize=8, zorder=6)
box(0.15, 0.2, 11.7, 7.0, "", "#fafafa", ls="--", z=0); ax.text(0.3, 7.0, "one `sim` process, one virtual clock: two MCUs advance together in 40 us quanta", fontsize=10, weight="bold", va="center")
for row, (y, name, enc) in enumerate(((4.1, "joint 1 (shoulder)", "pi/2"), (0.9, "joint 2 (elbow)", "pi"))):
    k = row + 1
    box(0.4, y, 4.0, 2.5, "", "#eef3fb", z=1); ax.text(0.55, y + 2.3, f"simulated STM32G474: {name}", fontsize=9, weight="bold", va="center")
    box(0.65, y + 1.35, 3.5, 0.75, f"firmware/joint, JOINT_ID={k}\nSimpleFOC angle loop", "#dbe7fa", bold=True, fs=8.5)
    box(0.65, y + 0.75, 1.65, 0.5, "TIM4 PWM", "#fff", fs=8); box(2.5, y + 0.75, 1.65, 0.5, "SPI1", "#fff", fs=8)
    box(0.65, y + 0.15, 3.5, 0.5, "FDCAN2", "#fff", fs=8)
    box(5.2, y + 1.3, 2.3, 0.8, f"plant_j{k}.py\nmotor equations", "#fde9cf", bold=True, fs=8.5)
    box(5.2, y + 0.4, 2.3, 0.7, f"enc_j{k}.py: MT6835,\nreads {enc} at rest", "#fff", fs=8)
    arrow((2.3, y + 1.0), (5.2, y + 1.55), "duty", off=(0.3, 0.12))
    arrow((4.15, y + 0.95), (5.2, y + 0.75), "", both=True)
    arrow((7.5, y + 1.7), (8.9, 3.9 if k == 1 else 3.3), "torque" if k == 1 else "torque", off=(0, 0.15))
    arrow((8.9, 3.6 if k == 1 else 3.0), (7.5, y + 0.75), "angle", off=(-0.2, -0.25))
box(8.9, 2.7, 2.7, 1.5, "MuJoCo 4.0.1\nworld.py: one two-link arm,\nsteps 40 us once both\ntorques are in", "#dff3e3", bold=True, fs=8.5)
box(8.9, 0.45, 2.7, 0.9, "can_master_2link.py\nCAN master", "#fff", fs=8.5)
for y in (4.5, 1.3): ax.plot([4.15, 4.6], [y, y], color="#222", lw=2.2)
ax.plot([4.6, 4.6], [4.5, 0.62], color="#222", lw=2.2); ax.plot([4.6, 8.9], [0.62, 0.62], color="#222", lw=2.2)
ax.text(6.7, 0.35, "one CAN bus: 0x201/0x202 targets in, 0x101/0x102 status out", ha="center", fontsize=8)
fig.tight_layout(); fig.savefig("sim/screenshots/two-link-diagram.png"); print("ok")
