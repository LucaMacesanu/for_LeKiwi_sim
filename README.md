# LeKiwi-sim (fork)

Forked from [SIGRobotics-UIUC/LeKiwi-sim](https://github.com/SIGRobotics-UIUC/LeKiwi-sim)
— all credit for the original Fusion→MuJoCo conversion and mesh work goes there. This
fork carries local physics/model fixes made while driving this specific MJCF from ROS 2
as part of [lekiwi-so101-ros2-mujoco](https://github.com/YOUR_GITHUB_USERNAME/lekiwi-so101-ros2-mujoco)
— see that repo for the ROS bridge code, full setup docs, and pitfalls.

## Usage
1. `pip install mujoco`
2. `python -m mujoco.viewer --mjcf=mjcf_lcmm_robot.xml`

## What's changed from upstream

Full detail is in `git log` (real commit messages, not just this summary) — worth
reading before touching the model further, several of these fixes replaced earlier
attempts that made things worse and were reverted:

- **Mass correction**: original model was ~4-5x heavier than the real robot (compared
  against known real component weights) — this was the actual root cause of most
  wheel-driving instability chased early on, not solver/friction/timestep settings.
  Fixed with a uniform 0.25 scale factor on every `<inertial mass=... fullinertia=...>`.
- **Real omni-wheel physics**: replaced the original single-collision-sphere-per-wheel
  approximation with 24 passive-hinge roller bodies (8 per wheel) so the wheels behave
  like real omni-wheels instead of simple spheres.
- **Wheel self-collision fix**: each wheel's motor-housing geom was in active
  penetrating contact with its own hub geom, generating a constraint force that
  overpowered any actuator torque and made the base appear completely locked. Fixed
  with `<contact><exclude>` pairs.
- **Arm reparented under the base**: was two visually-disconnected top-level bodies
  under `world`; now a true body-subtree reparent (an `<equality><weld>` was tried
  first, destabilized driving, reverted in favor of this).
- **Floor + freejoint + target block**: added for a standalone driving/manipulation
  practice environment (no ROS required to use it).
- **Arm-floor collision proxies**: so the arm can't phase through the floor once it
  could actually reach it.
- **Gripper (`Jaw` body) rebuilt from scratch**: the original hand-authored mesh/joint
  never worked correctly regardless of tuning. Replaced wholesale with the official
  SO-ARM100 gripper mesh/joint (`moving_jaw_so101_v1`), including a numerically-derived
  body position correction to keep the hinge point aligned after the swap.
- **`Wrist_Roll` range re-centered**: the real robot's `wrist_roll` has a single-sided
  mechanical hard stop (found by hand at -105°, not a symmetric ±160° as first
  assumed) — this model's joint/ctrl range was re-centered to match.
- Actuator gain/damping tuning (`kp`, `damping`, `forcerange`) on several joint
  classes, arrived at empirically via live leader-arm-driven testing, not simulation
  alone.

## Known unresolved issues

- Sim occasionally shakes/vibrates at the resting L-shape pose, most visibly on the
  `Rotation`/`Jaw` joints but originating from `Pitch` (highest gravity load in this
  pose). Bounded/cosmetic so far (not diverging to NaN) — not fully root-caused.
  Candidate next step if revisited: joint-level damping specifically on `Pitch`
  (never tried; damping was added to `Rotation`/`Jaw` first based on where the
  shaking was most *visible*, not where it *originates*).
- Wheel-drive gain (`GAIN_LINEAR`/`GAIN_ANGULAR`, in the ROS bridge's `bridge_node.py`,
  not this file) has only been headless-tested, not live-driven by a human yet, as of
  the last change to this model.
- Forward driving drifts in heading over a sustained hold — expected 3-wheel-omni
  kinematics behavior (the wheel nearest the direction of travel inherently carries
  more load), correctable by the driver, not something to chase out via gain tuning.

## Debugging note worth keeping

For any body whose actual collision shape lives in joint/geom offsets rather than the
body's own `pos` (e.g. every roller body here), **`d.xpos` will always report the
parent's origin regardless of joint angle** — this cost real debugging time once. Use
`d.geom_xpos` and `mj_contactForce` to verify geometry/contacts, never `d.xpos`, for
bodies like this.

## Converting from Fusion to MuJoCo (upstream notes, unchanged)
- Using this plugin: https://github.com/bionicdl-sustech/ACDC4Robot
- For `AttributeError: module 'time' has no attribute 'stop'` use
https://github.com/bionicdl-sustech/ACDC4Robot/issues/1
- Make sure to remove all nested components in CAD
- Simplify large meshes(like omniwheels) using a [mesh simplifier](https://myminifactory.github.io/Fast-Quadric-Mesh-Simplification/) if mujoco complains about too many faces
