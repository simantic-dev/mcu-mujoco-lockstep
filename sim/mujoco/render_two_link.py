# GIF/MP4 of two-link runs (run_two_link.py), all cases side by side on one virtual clock: MuJoCo's own render of each case's arm set to the
# logged joint angles (green ghost = commanded pose), the shoulder and elbow angles underneath. Nothing is re-simulated.
# Usage: python sim/mujoco/render_two_link.py [out_name] [case ...]   (default: every case in two_link_cases.py that has a log)
import json, math, os, re, subprocess, sys, textwrap
import numpy as np, mujoco
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(ROOT, "sim", "peers"))
from two_link_cases import CASES
from can_master_2link import STEPS
from angles import wrapped
OUT = sys.argv[1] if len(sys.argv) > 1 else "two-link-cases"
WANT = sys.argv[2:]
R = json.load(open(os.path.join(ROOT, "sim/out/two/results.json")))
cases = [c for c in CASES if (not WANT or c["name"] in WANT) and os.path.exists(os.path.join(ROOT, "sim/out/two", c["name"], "sim.log"))]
def target(t):
    tg = (0.0, 0.0)
    for s, p in STEPS:
        if t >= s / 1e6: tg = p
    return tg
STRIDE = 8
big = len(cases) == 1
W, H = (640, 480) if big else (300, 225)
data = {}
for c in cases:
    d = os.path.join(ROOT, "sim/out/two", c["name"])
    rows = {1: {}, 2: {}}
    for l in open(os.path.join(d, "sim.log"), errors="ignore"):
        m = re.search(r"J([12]) t=(\S+) q=(\S+) qr=(\S+)", l)
        if m: rows[int(m.group(1))][round(float(m.group(2)), 4)] = (float(m.group(3)), float(m.group(4)))
    ts = sorted(set(rows[1]) & set(rows[2]))
    t = np.array(ts); q1 = np.array([rows[1][x][0] for x in ts]); q2 = np.array([rows[2][x][0] for x in ts])
    r1 = np.array([rows[1][x][1] for x in ts]); r2 = np.array([rows[2][x][1] for x in ts])
    m = mujoco.MjModel.from_xml_path(os.path.join(d, "arm.xml")); dd = mujoco.MjData(m); ren = mujoco.Renderer(m, H, W)
    lash = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "rotor1_j") >= 0
    adr = lambda n: int(m.joint(n).qposadr[0])
    g1, g2 = (mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n) for n in ("ghost1", "ghost2"))
    imgs = []
    for i in range(0, len(t), STRIDE):
        dd.qpos[adr("j1")], dd.qpos[adr("j2")] = q1[i], q2[i]
        if lash: dd.qpos[adr("rotor1_j")], dd.qpos[adr("rotor2_j")] = r1[i], r2[i]
        a, b = target(t[i])
        m.body_quat[g1] = [math.cos(a / 2), 0, math.sin(a / 2), 0]; m.body_quat[g2] = [math.cos(b / 2), 0, math.sin(b / 2), 0]
        mujoco.mj_forward(m, dd); ren.update_scene(dd, camera="side"); imgs.append(ren.render().copy())
    ren.close()
    data[c["name"]] = dict(t=t, w1=wrapped(q1, math.pi / 2), w2=wrapped(q2), imgs=imgs)
n = len(cases); cols = 1 if big else 4; rows_n = math.ceil(n / cols)
fig = plt.figure(figsize=(9, 7.6) if big else (15.6, 4.9 * rows_n + 0.5), dpi=80)
pw, ph = 1 / cols, (1 - 0.06) / rows_n
art = []
for k, c in enumerate(cases):
    r_, c_ = divmod(k, cols); x0, y0 = c_ * pw, 1 - 0.06 - (r_ + 1) * ph
    ax = fig.add_axes([x0 + 0.006, y0 + ph * 0.40, pw - 0.012, ph * 0.43]); ax.axis("off"); im = ax.imshow(data[c["name"]]["imgs"][0])
    fig.text(x0 + 0.01, y0 + ph - 0.008, c["label"], fontsize=12 if big else 10, weight="bold", va="top")
    fig.text(x0 + 0.01, y0 + ph - (0.035 if big else 0.042), "\n".join(textwrap.wrap(c["params"], 90 if big else 46)), fontsize=8 if big else 7.2, va="top", color="#333")
    tr = fig.add_axes([x0 + (0.07 if big else 0.03), y0 + ph * 0.115, pw - (0.1 if big else 0.045), ph * 0.25]); tr.set_xlim(0, 17); tr.set_ylim(-3.5, 4.9); tr.tick_params(labelsize=7 if big else 6)
    tt = data[c["name"]]["t"]
    tr.plot(tt, [target(x)[0] for x in tt], "--", color="C0", lw=0.7); tr.plot(tt, [target(x)[1] for x in tt], "--", color="C1", lw=0.7)
    l1, = tr.plot([], [], color="C0", lw=1.1, label="shoulder"); l2, = tr.plot([], [], color="C1", lw=1.1, label="elbow")
    if big or k == 0: tr.legend(loc="lower right", fontsize=6.5)
    res = R.get(c["name"], {})
    if "phase1" not in res: cap = "a joint did not reach 'ready'"
    else: cap = "  ".join(f"{nm}: {'held' if res[p]['held'] else 'NOT held'}" for nm, p in (("out", "phase1"), ("up", "phase2"), ("up, elbow bent", "phase3")))
    fig.text(x0 + 0.01, y0 + 0.004, cap, fontsize=8 if big else 7, va="bottom", family="monospace")
    art.append((im, l1, l2, c["name"]))
head = fig.text(0.01, 0.99, "", fontsize=11, weight="bold", va="top")
fig.text(0.01, 0.965 if big else 0.975, "two simulated STM32G474s, one firmware each, on one CAN bus; MuJoCo arm; playback of captured runs, virtual time", fontsize=7.5, va="top", color="gray")
frames = []
nf = min(len(v["imgs"]) for v in data.values())
for i in range(nf):
    for im, l1, l2, nm in art:
        v = data[nm]; im.set_data(v["imgs"][i]); j = min(i * STRIDE, len(v["t"]) - 1)
        l1.set_data(v["t"][:j + 1], v["w1"][:j + 1]); l2.set_data(v["t"][:j + 1], v["w2"][:j + 1])
    tnow = data[cases[0]["name"]]["t"][min(i * STRIDE, len(data[cases[0]["name"]]["t"]) - 1)]
    phase = "power-up spin, then calibration" if tnow < 8.06 else "holding the hanging pose" if tnow < STEPS[0][0] / 1e6 else "arm out" if tnow < STEPS[1][0] / 1e6 else "arm straight up (inverted)" if tnow < STEPS[2][0] / 1e6 else "straight up, elbow bent 0.35 rad"
    head.set_text(f"Two-link arm, firmware on simulated MCUs + MuJoCo: t = {tnow:5.2f} s   ({phase})")
    fig.canvas.draw(); frames.append(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy())
h, w, _ = frames[0].shape
out = os.path.join(ROOT, "sim/screenshots"); os.makedirs(out, exist_ok=True)
p = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", "12.5", "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "26", os.path.join(out, OUT + ".mp4")], stdin=subprocess.PIPE)
for f in frames: p.stdin.write(f.tobytes())
p.stdin.close(); p.wait()
g = [Image.fromarray(f).convert("P", palette=Image.ADAPTIVE, colors=64) for f in frames]
g[0].save(os.path.join(out, OUT + ".gif"), save_all=True, append_images=g[1:], duration=80, loop=0, optimize=True)
print(len(frames), "frames", w, "x", h)
