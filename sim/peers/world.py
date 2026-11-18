# The one MuJoCo model shared by the two joints' plant peers (plant2.py). Each plant peer, every 40 us of virtual time, submits its motor
# torque; when both joints have submitted, MuJoCo steps once. A plant reads the state as it is when it ticks, so a joint can see the other
# joint's motion up to one tick (40 us) late or early depending on which machine the simulator ran first; the order is fixed by the
# simulator's scheduling, and sim/mujoco/run_two_link.py checks that two runs give identical logs.
import os
import mujoco


class World:
    def __init__(self, xml):
        self.m = mujoco.MjModel.from_xml_path(xml)
        self.d = mujoco.MjData(self.m)
        self.n = 2
        lash = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, "rotor1_j") >= 0
        self.rotor = [int(self.m.joint(f"rotor{k}_j" if lash else f"j{k}").qposadr[0]) for k in (1, 2)]
        self.load = [int(self.m.joint(f"j{k}").qposadr[0]) for k in (1, 2)]
        self.rotor_v = [int(self.m.joint(f"rotor{k}_j" if lash else f"j{k}").dofadr[0]) for k in (1, 2)]
        self.load_v = [int(self.m.joint(f"j{k}").dofadr[0]) for k in (1, 2)]
        self.tau = [0.0] * self.n
        self.have = [False] * self.n
        self.steps = 0

    def submit(self, k, torque):
        self.tau[k] = torque
        self.have[k] = True
        if all(self.have):
            self.d.ctrl[:] = self.tau
            mujoco.mj_step(self.m, self.d)
            self.have = [False] * self.n
            self.steps += 1


_WORLD = None


def get(xml=None):
    global _WORLD
    if _WORLD is None:
        _WORLD = World(xml or os.environ["ARM_XML"])
    return _WORLD
