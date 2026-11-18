# Builds the arm model for one case of sim/mujoco/run_cases.py.
#   backlash == 0: the single-hinge arm.xml, with the payload, viscous damping and Coulomb friction changed.
#   backlash  > 0: the motor's rotor and the load are two hinges joined through a gap of `backlash` rad. The gap is a
#                  MuJoCo fixed tendon (rotor angle - load angle) with limited="true" and range +-backlash/2: MuJoCo's own
#                  constraint solver stops the relative motion at the ends, which is the standard dead-zone backlash
#                  (Nordin and Gutman, Automatica 38(10), 2002) with the stops as soft constraints.
#                  MODELED SIMPLIFICATION: stop stiffness solreflimit = (LASH_TIMECONST, 1), a free parameter.
# Contacts are off (arm.xml): the arm touches nothing, and the rotor disc overlapping the link at the pivot would otherwise collide with it.
# Torque acts on the rotor (armature 7.4e-6 kg*m^2, the GL40's rotor inertia, as in arm.xml); gravity, payload, damping and friction are on the load.
import os, re

HERE = os.path.dirname(os.path.abspath(__file__))
LASH_TIMECONST = 0.0002  # s; the constraint is stiffer than 1 ms and slower than the 40 us step (MuJoCo keeps rows with timeconst*dampratio >= step)


def make(payload=0.07, damping=0.01, frictionloss=0.0, backlash=0.0, gravity=True):
    s = open(os.path.join(HERE, "arm.xml")).read()
    if not gravity:  # hinge axis vertical, the arm swings in a horizontal plane: no gravity torque on the joint
        s = s.replace('gravity="0 0 -9.81"', 'gravity="0 0 0"')
    s = s.replace('size="0.02" mass="0.07"', f'size="0.02" mass="{payload}"')
    if backlash <= 0:
        return s.replace('damping="0.01" armature="7.4e-6"', f'damping="{damping}" armature="7.4e-6" frictionloss="{frictionloss}"')
    # two bodies: a rotor disc (with a marker, so the gap is visible in a render) and the load link
    rotor = '''<body name="rotor">
      <joint name="rotor_j" type="hinge" axis="0 1 0" armature="7.4e-6"/>
      <geom type="cylinder" fromto="0 -0.012 0 0 0.012 0" size="0.04" mass="0.01" rgba="0.55 0.55 0.6 1"/>
      <geom type="box" pos="0 0.014 -0.034" size="0.006 0.003 0.008" mass="0.0001" rgba="1 0.25 0.25 1"/>
    </body>
    '''
    s = s.replace('<body name="link" pos="0 0 0">', rotor + '<body name="link" pos="0 0 0">')
    s = s.replace('<joint name="hinge" type="hinge" axis="0 1 0" damping="%s" armature="7.4e-6"/>' % "0.01",
                  f'<joint name="load_j" type="hinge" axis="0 1 0" damping="{damping}" frictionloss="{frictionloss}"/>')
    s = s.replace('<actuator>\n    <motor joint="hinge"', f'''<tendon>
    <fixed name="lash" limited="true" range="{-backlash/2} {backlash/2}" solreflimit="{LASH_TIMECONST} 1">
      <joint joint="rotor_j" coef="1"/>
      <joint joint="load_j" coef="-1"/>
    </fixed>
  </tendon>
  <actuator>
    <motor joint="rotor_j"''')
    assert "rotor_j" in s and "load_j" in s and "lash" in s
    return s
