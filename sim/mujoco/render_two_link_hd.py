# HD playback of one two-link run (run_two_link.py <case>): MuJoCo's own render of the arm, the joint angles, both MCUs' USB consoles and
# the CAN traffic (commands from the master, and the angle and speed each joint's firmware reports in its status frames).
# Nothing is re-simulated. Usage: python sim/mujoco/render_two_link_hd.py [case] [out_name]
import math, os, re, subprocess, sys
import numpy as np, mujoco
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(ROOT, "sim", "peers"))
from can_master_2link import STEPS
from angles import wrapped
CASE = sys.argv[1] if len(sys.argv) > 1 else "ideal"
OUT = sys.argv[2] if len(sys.argv) > 2 else "two-link"
d = os.path.join(ROOT, "sim/out/two", CASE)
def target(t):
    tg = (0.0, 0.0)
    for s, p in STEPS:
        if t >= s / 1e6: tg = p
    return tg
rows = {1: {}, 2: {}}
can = []
for l in open(os.path.join(d, "sim.log"), errors="ignore"):
    m = re.search(r"J([12]) t=(\S+) q=(\S+) qr=(\S+)", l)
    if m: rows[int(m.group(1))][round(float(m.group(2)), 4)] = (float(m.group(3)), float(m.group(4))); continue
    m = re.search(r"ARM_CMD t=(\S+) shoulder (\S+) elbow (\S+) rad", l)
    if m: can.append((float(m.group(1)), f"master -> 0x201/0x202  targets {float(m.group(2)):+.3f} / {float(m.group(3)):+.3f} rad")); continue
    m = re.search(r"STATUS j([12]) t=(\S+) angle=(\S+) speed=(\S+)", l)
    if m: can.append((float(m.group(2)), f"joint {m.group(1)} -> 0x10{m.group(1)}  angle {m.group(3)} rad  speed {m.group(4)} rad/s"))
can.sort()
con = {1: [], 2: []}
for l in open(os.path.join(d, "uart.log"), errors="ignore"):
    m = re.match(r"\[j([12])\] \[([\d.]+)s\] \(usb\) (.*)", l.strip())
    if m and not m.group(3).startswith("TIM"): con[int(m.group(1))].append((float(m.group(2)), m.group(3)))
ts = sorted(set(rows[1]) & set(rows[2]))
t = np.array(ts); q1, q2 = (np.array([rows[k][x][0] for x in ts]) for k in (1, 2)); r1, r2 = (np.array([rows[k][x][1] for x in ts]) for k in (1, 2))
w1, w2 = wrapped(q1, math.pi / 2), wrapped(q2)
m = mujoco.MjModel.from_xml_path(os.path.join(d, "arm.xml")); dd = mujoco.MjData(m)
m.vis.global_.offwidth, m.vis.global_.offheight = 1280, 960
ren = mujoco.Renderer(m, 720, 900)
lash = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "rotor1_j") >= 0
adr = lambda n: int(m.joint(n).qposadr[0])
g1, g2 = (mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n) for n in ("ghost1", "ghost2"))
fig = plt.figure(figsize=(16, 9), dpi=100)
ax = fig.add_axes([0.005, 0.03, 0.52, 0.85]); ax.axis("off"); im = ax.imshow(np.zeros((720, 900, 3), np.uint8))
head = fig.text(0.01, 0.975, "", fontsize=17, weight="bold", va="top")
fig.text(0.01, 0.925, "Two simulated STM32G474s, each running the joint firmware, on one CAN bus; one MuJoCo two-link arm. Playback of a captured run, in virtual time.", fontsize=10.5, color="#555", va="top")
tr = fig.add_axes([0.575, 0.66, 0.405, 0.22]); tr.set_xlim(0, t[-1]); tr.set_ylim(-3.5, 4.9); tr.tick_params(labelsize=9)
tr.plot(t, [target(x)[0] for x in t], "--", color="C0", lw=0.9); tr.plot(t, [target(x)[1] for x in t], "--", color="C1", lw=0.9)
l1, = tr.plot([], [], color="C0", lw=1.6, label="shoulder (MuJoCo)"); l2, = tr.plot([], [], color="C1", lw=1.6, label="elbow (MuJoCo)")
tr.legend(loc="lower right", fontsize=9); tr.set_ylabel("rad", fontsize=10); tr.set_xlabel("virtual time, s", fontsize=9)
def pane(y, h, title, color):
    a = fig.add_axes([0.56, y, 0.435, h]); a.set_xticks([]); a.set_yticks([])
    for s in a.spines.values(): s.set_color(color); s.set_linewidth(1.5)
    fig.text(0.565, y + h - 0.008, title, fontsize=11, weight="bold", color=color, va="top")
    return fig.text(0.565, y + h - 0.042, "", fontsize=9.5, family="monospace", va="top")
txt1 = pane(0.385, 0.2, "MCU 1 (shoulder), USB console", "C0")
txt2 = pane(0.17, 0.2, "MCU 2 (elbow), USB console", "C1")
txtc = pane(0.01, 0.145, "CAN bus", "#444")
STRIDE = 8
frames = []
for i in range(0, len(t), STRIDE):
    dd.qpos[adr("j1")], dd.qpos[adr("j2")] = q1[i], q2[i]
    if lash: dd.qpos[adr("rotor1_j")], dd.qpos[adr("rotor2_j")] = r1[i], r2[i]
    a, b = target(t[i])
    m.body_quat[g1] = [math.cos(a / 2), 0, math.sin(a / 2), 0]; m.body_quat[g2] = [math.cos(b / 2), 0, math.sin(b / 2), 0]
    mujoco.mj_forward(m, dd); ren.update_scene(dd, camera="side"); im.set_data(ren.render())
    l1.set_data(t[:i + 1], w1[:i + 1]); l2.set_data(t[:i + 1], w2[:i + 1])
    now = t[i]
    phase = "power-up spin, then calibration" if now < 8.06 else "holding the hanging pose" if now < STEPS[0][0] / 1e6 else "arm out" if now < STEPS[1][0] / 1e6 else "straight up (inverted)" if now < STEPS[2][0] / 1e6 else "straight up, elbow bent 0.35 rad"
    head.set_text(f"Firmware on simulated MCUs + MuJoCo    t = {now:5.2f} s    {phase}")
    for k, tx in ((1, txt1), (2, txt2)):
        tx.set_text("\n".join(f"[{s:6.3f}s] {x}" for s, x in [c for c in con[k] if c[0] <= now][-6:]))
    txtc.set_text("\n".join(f"[{s:6.3f}s] {x}" for s, x in [c for c in can if c[0] <= now][-4:]))
    fig.canvas.draw(); frames.append(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy())
ren.close()
h, w, _ = frames[0].shape
out = os.path.join(ROOT, "sim/screenshots")
p = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", "12.5", "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", os.path.join(out, OUT + ".mp4")], stdin=subprocess.PIPE)
for f in frames: p.stdin.write(f.tobytes())
p.stdin.close(); p.wait()
g = [Image.fromarray(f).convert("P", palette=Image.ADAPTIVE, colors=128) for f in frames]
g[0].save(os.path.join(out, OUT + ".gif"), save_all=True, append_images=g[1:], duration=80, loop=0, optimize=True)
print(len(frames), "frames", w, "x", h, os.path.getsize(os.path.join(out, OUT + ".gif")) // 1024, "KiB")
