# Runs every case in two_link_cases.py through two firmware instances in sim (the two-link scenario) and scores each from MuJoCo's true joint angles.
# Usage: [SIM_LOCK=<lock file> to queue runs behind other heavy jobs] VENV=<venv> python sim/mujoco/run_two_link.py [case ...]   -> sim/out/two/<name>/{arm.xml,sim.log,uart.log}, sim/out/two/results.json
import json, math, os, re, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(ROOT, "sim", "peers"))
import two_link_xml
from two_link_cases import CASES
from can_master_2link import STEPS
END_S = 17.0

def phases():
    ends = [s / 1e6 for s, _ in STEPS[1:]] + [END_S]
    return [(s / 1e6, e, tg) for (s, tg), e in zip(STEPS, ends)]

def score(rows):
    out = {}
    for k, (s, e, tg) in enumerate(phases(), 1):
        ph = {}
        for j in (0, 1):
            seg = [(t, q) for t, q in rows[j] if s <= t <= e]
            last = [q - tg[j] for t, q in seg if t >= e - 0.5]
            ph[f"j{j + 1}"] = dict(target=tg[j], mean_err=sum(last) / len(last), max_abs_err=max(abs(x) for x in last), pp=max(last) - min(last))
        ph["held"] = all(ph[f"j{j + 1}"]["max_abs_err"] < 0.1 for j in (0, 1))
        out[f"phase{k}"] = ph
    return out

def main():
    venv = os.environ["VENV"]; sp = [os.path.join(venv, "lib", d, "site-packages") for d in os.listdir(os.path.join(venv, "lib"))][0]
    want = sys.argv[1:] or [c["name"] for c in CASES]
    os.makedirs(os.path.join(ROOT, "sim/out/two"), exist_ok=True)
    path = os.path.join(ROOT, "sim/out/two/results.json"); res = json.load(open(path)) if os.path.exists(path) else {}
    for c in CASES:
        if c["name"] not in want: continue
        d = os.path.join(ROOT, "sim/out/two", c["name"]); os.makedirs(d, exist_ok=True)
        open(os.path.join(d, "arm.xml"), "w").write(two_link_xml.make(**c["arm"]))
        env = dict(os.environ, PYTHONPATH=sp, ARM_XML=os.path.join(d, "arm.xml"))
        env.update({k: str(v) for k, v in c["env"].items()})
        t0 = time.time()
        cmd = (["lockf", "-k", os.environ["SIM_LOCK"]] if os.environ.get("SIM_LOCK") else []) + ["timeout", "300", "sim", "run", "--scenario", "sim/scenario-2link.yaml", "--usb-cdc-console", "j1.usb", "--usb-cdc-console", "j2.usb",
               "--timeout", str(END_S), "--ascii", "--expect", "JOINT 1 ready", "--expect", "JOINT 2 ready"]
        if os.environ.get("RESCORE") and os.path.exists(os.path.join(d, "sim.log")):
            r = type("R", (), {"returncode": 0})
        else:
            r = subprocess.run(cmd, cwd=ROOT, env=env, stdout=open(os.path.join(d, "uart.log"), "w"), stderr=open(os.path.join(d, "sim.log"), "w"))
        rows = [[], []]
        for l in open(os.path.join(d, "sim.log"), errors="ignore"):
            m = re.search(r"J([12]) t=(\S+) q=(\S+) qr=(\S+)", l)
            if m: rows[int(m.group(1)) - 1].append((float(m.group(2)), float(m.group(3))))
        uart = open(os.path.join(d, "uart.log"), errors="ignore").read()
        ready = [re.search(rf"\[([\d.]+)s\] \(usb\) JOINT {j} ready", uart) for j in (1, 2)]
        res[c["name"]] = dict(exit=r.returncode, wall_s=round(time.time() - t0, 1), ready_s=[float(x.group(1)) if x else None for x in ready])
        if all(ready) and rows[0] and rows[1]: res[c["name"]].update(score(rows))
        print(c["name"], json.dumps(res[c["name"]])[:300], flush=True)
        json.dump(res, open(path, "w"), indent=1)
main()
