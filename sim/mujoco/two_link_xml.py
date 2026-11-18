# Two-link arm in a vertical plane (shoulder j1 on the world, elbow j2 on link 1), each joint driven by its own motor, hinge axes along y,
# hanging straight down at q = 0. Each joint is either one hinge (backlash == 0) or a rotor hinge and a load hinge joined through a gap of
# `backlash` rad: a MuJoCo fixed tendon (rotor angle - load angle) with limited range +-backlash/2, as in arm_xml.py.
# The motor torque (data.ctrl) acts on the rotor, or on the single hinge. j2 is the angle of link 2 relative to link 1.
# Contacts are off: the arm touches nothing, and the rotor discs overlapping at the pivots would otherwise collide and jam the joints.
# MODELED SIMPLIFICATION: LASH_TIMECONST (stop stiffness) is a free parameter; link masses and lengths are chosen so gravity at the shoulder
# stays under the GL40's 0.25 N*m rated torque (0.235 N*m at the shoulder with the arm horizontal).
L1, L2 = 0.3, 0.3
LASH_TIMECONST = 0.0002


def make(tip_mass=0.02, damping=0.01, frictionloss=0.0, backlash=0.0, link_mass=0.02):
    lash = backlash > 0
    def joint(k, parent_geom):
        # the load-side hinge of joint k
        name = f"j{k}"
        return f'<joint name="{name}" type="hinge" axis="0 1 0" damping="{damping}" frictionloss="{frictionloss}"' + ("" if lash else ' armature="7.4e-6"') + "/>"
    def rotor(k):
        if not lash: return ""
        return f'''<body name="rotor{k}">
        <joint name="rotor{k}_j" type="hinge" axis="0 1 0" armature="7.4e-6"/>
        <geom type="cylinder" fromto="0 -0.012 0 0 0.012 0" size="0.035" mass="0.01" rgba="0.55 0.55 0.6 1"/>
        <geom type="box" pos="0 0.014 -0.03" size="0.005 0.003 0.007" mass="0.0001" rgba="1 0.25 0.25 1"/>
      </body>
      '''
    tendons = "".join(f'''<fixed name="lash{k}" limited="true" range="{-backlash/2} {backlash/2}" solreflimit="{LASH_TIMECONST} 1">
      <joint joint="rotor{k}_j" coef="1"/><joint joint="j{k}" coef="-1"/></fixed>
    ''' for k in (1, 2)) if lash else ""
    motors = "".join(f'<motor joint="{"rotor%d_j" % k if lash else "j%d" % k}" gear="1" ctrllimited="true" ctrlrange="-1 1"/>' for k in (1, 2))
    return f'''<mujoco model="two_link">
  <option timestep="0.00004" gravity="0 0 -9.81" integrator="implicitfast"><flag contact="disable"/></option>
  <visual><headlight ambient="0.55 0.55 0.55" diffuse="0.7 0.7 0.7" specular="0 0 0"/><global offwidth="960" offheight="720"/></visual>
  <asset><texture type="skybox" builtin="gradient" rgb1="0.35 0.45 0.6" rgb2="0.05 0.07 0.1" width="32" height="512"/></asset>
  <worldbody>
    <light pos="0.5 1 0.5" dir="-0.3 -1 -0.5"/>
    <camera name="side" pos="0 1.55 -0.02" xyaxes="-1 0 0 0 0 1"/>
    <geom name="mount" type="box" pos="0 -0.03 0" size="0.06 0.015 0.06" rgba="0.5 0.5 0.55 1" contype="0" conaffinity="0"/>
    <body name="ghost1">
      <geom type="capsule" fromto="0 0 0 0 0 -{L1}" size="0.008" rgba="0.2 0.95 0.3 0.45" contype="0" conaffinity="0" mass="0"/>
      <body name="ghost2" pos="0 0 -{L1}">
        <geom type="capsule" fromto="0 0 0 0 0 -{L2}" size="0.008" rgba="0.2 0.95 0.3 0.45" contype="0" conaffinity="0" mass="0"/>
        <geom type="sphere" pos="0 0 -{L2}" size="0.011" rgba="0.2 0.95 0.3 0.6" contype="0" conaffinity="0" mass="0"/>
      </body>
    </body>
    {rotor(1)}<body name="link1">
      {joint(1, "")}
      <geom type="capsule" fromto="0 0 0 0 0 -{L1}" size="0.01" mass="{link_mass}" rgba="0.85 0.85 0.9 1"/>
      {rotor(2)}<body name="link2" pos="0 0 -{L1}">
        {joint(2, "")}
        <geom type="capsule" fromto="0 0 0 0 0 -{L2}" size="0.01" mass="{link_mass}" rgba="0.75 0.8 0.95 1"/>
        <geom type="sphere" pos="0 0 -{L2}" size="0.02" mass="{tip_mass}" rgba="0.95 0.45 0.15 1"/>
      </body>
    </body>
  </worldbody>
  {"<tendon>" + tendons + "</tendon>" if lash else ""}
  <actuator>{motors}</actuator>
</mujoco>
'''
