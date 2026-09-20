# Spot Locomotion V3 — Gazebo Sim-to-Sim Deployment Update

## Scope

This document continues from the previous Spot V3 engineering handoff and records the work completed after the V3 PPO actuator root-cause investigation.

The previous milestone had already established the approved MuJoCo V3 locomotion baseline:

- Isaac-derived 31.6 kg rigid-body model
- MjLab 1.5.3 / MuJoCo 3.10
- 500 Hz physics
- 50 Hz policy
- native position actuator semantics
- `Kp = 300`
- `Kd = 40`
- hip effort limit `±45 Nm`
- knee effort limit `±115 Nm`
- simple flat-terrain PPO task
- stable and natural locomotion in MuJoCo

The work described here focused on transferring that system into ROS 2 + Gazebo and validating the complete MuJoCo → ROS 2/Gazebo sim-to-sim deployment pipeline.

---

# 1. Gazebo V3 model integration

The existing ROS 2 / Gazebo workspace was updated to use the V3 robot dynamics instead of the older V2 model.

Workspace:

```text
~/Projects/spot_ros2_ws
```

Platform:

```text
Ubuntu 24.04
ROS 2 Jazzy
Gazebo Harmonic
gz_ros2_control
```

The Gazebo robot uses the same approved V3 rigid-body properties as the MuJoCo training model.

## V3 total mass

```text
31.5999994 kg
```

Main link masses:

```text
Body:       16.707651 kg
Hip:         1.1368834 kg each
Upper leg:   2.2562037 kg each
Lower leg:   0.3300000 kg each
```

The existing Spot geometry, link hierarchy, joint names, and nominal pose were preserved.

## Joint order

```text
fl_hx, fl_hy, fl_kn,
fr_hx, fr_hy, fr_kn,
hl_hx, hl_hy, hl_kn,
hr_hx, hr_hy, hr_kn
```

## Nominal pose

```text
HX =  0.00 rad
HY =  1.04 rad
KN = -1.80 rad
```

for all four legs.

---

# 2. Gazebo low-level controller architecture

The deployment boundary remains an effort command interface.

The intended control path is:

```text
/cmd_vel
    ↓
PPO policy @ 50 Hz
    ↓
12 joint-position references
    ↓
q_ref = q_home + action_scale × policy_action
    ↓
low-level joint controller
    ↓
tau = Kp(q_ref - q) - Kd qdot
    ↓
effort clamp
    ↓
ros2_control effort interface
    ↓
gz_ros2_control
    ↓
Gazebo
```

The approved controller constants were carried over unchanged from the MuJoCo result:

```text
Kp = 300
Kd = 40
Ki = 0
```

Effort limits:

```text
HX / HY: ±45 Nm
Knee:    ±115 Nm
```

The ROS controller receives:

```text
position reference = q_ref
velocity reference = 0
```

The policy output remains a joint-position offset command, not a torque command.

---

# 3. Major Gazebo discovery: 500 Hz controller instability

The first important sim-to-sim problem appeared immediately when the V3 controller was tested in Gazebo.

Initial Gazebo configuration:

```text
Gazebo physics:       500 Hz
ros2_control PID:     500 Hz
PPO policy target:     50 Hz
Kp:                   300
Kd:                    40
Hip effort limit:     ±45 Nm
Knee effort limit:   ±115 Nm
```

With a fixed nominal standing reference, the robot did not remain stable.

Observed behavior:

- violent leg motion
- repeated effort saturation
- rapid direction reversals
- high-frequency limit-cycle-like behavior
- eventual fall

A representative unstable joint-state sample showed knee velocities around 8–14 rad/s while efforts repeatedly reached exactly:

```text
hips:  ±45 Nm
knees: ±115 Nm
```

This confirmed that the problem was not caused by PPO. The robot was unstable even under a deterministic fixed nominal joint reference.

---

# 4. Why identical gains behaved differently in MuJoCo and Gazebo

This exposed an important numerical difference between the two simulators.

In MuJoCo, the V3 PPO policy uses a native built-in position actuator together with the `implicitfast` integrator. The actuator damping is therefore incorporated into MuJoCo's implicit dynamics treatment.

Gazebo, in contrast, applies the PD law externally through `ros2_control` as an effort command.

The nominal control equation is the same:

```text
tau = Kp(q_ref - q) - Kd qdot
```

but the numerical implementation is not equivalent.

For the Spot knee, the apparent inertia is very small, roughly:

```text
~0.014 kg·m²
```

At:

```text
Kd = 40
qdot = 10 rad/s
```

the derivative term alone asks for approximately:

```text
-400 Nm
```

before saturation.

With the knee clamped to `±115 Nm`, a 2 ms control interval can still cause a very large velocity change:

```text
Δqdot ≈ (115 / 0.014) × 0.002
       ≈ 16.4 rad/s
```

This makes it possible for the explicit discrete controller to overshoot the velocity sign from one update to the next, creating rail-to-rail effort chatter.

This was the key reason the same nominal controller could be stable in MuJoCo but unstable in Gazebo.

---

# 5. Gazebo rate experiment: 500 Hz → 2000 Hz

Instead of changing the approved gains, the Gazebo integration rate was increased.

New configuration:

```text
Gazebo physics:       2000 Hz
ros2_control PID:     2000 Hz
PPO policy target:      50 Hz
Kp:                    300
Kd:                     40
Hip effort limit:      ±45 Nm
Knee effort limit:    ±115 Nm
```

This reduced the low-level time step from:

```text
2.0 ms → 0.5 ms
```

No gain or torque-limit changes were made.

Result:

```text
500 Hz physics + control:
300 / 40
±45 / ±115 Nm
→ unstable oscillation

2000 Hz physics + control:
300 / 40
±45 / ±115 Nm
→ firm stable standing
```

This was a major result.

It showed that the Gazebo failure was caused by the numerical implementation/rate combination rather than by the V3 model, torque limits, or the approved damping gain.

Because both Gazebo physics and controller-manager rates were changed together, the result should be interpreted as a validated combined Gazebo baseline rather than a proof that only one of those two rates was responsible.

No further split experiment was considered necessary because the 2000 Hz configuration worked reliably.

---

# 6. Stable standing verification

After increasing Gazebo and `ros2_control` to 2000 Hz, the robot stood firmly at the nominal reference.

Settled joint velocities were essentially zero:

```text
~1e-7 rad/s
```

Measured efforts remained well within limits.

Representative settled values included:

```text
FL knee: +16.23 Nm
FR knee: +11.59 Nm
HL knee: +27.31 Nm
HR knee: +20.20 Nm
```

The measured efforts also matched the expected proportional term almost exactly.

Example:

```text
FL knee q      = -1.854097 rad
FL knee q_ref  = -1.800000 rad
position error = +0.054097 rad

300 × 0.054097 ≈ 16.229 Nm
```

which matched the reported joint effort.

This verified that the Gazebo controller was implementing the expected low-level control equation correctly at equilibrium.

---

# 7. Short dynamic sanity check

A synchronized knee-reference test was then performed.

Nominal knee reference:

```text
-1.80 rad
```

Test knee reference:

```text
-1.70 rad
```

All four knees were moved together by:

```text
+0.10 rad
```

and then returned to nominal.

The robot remained stable and the motion was clean.

There was no renewed saturation chatter or obvious controller instability.

At that point the Gazebo plant and low-level controller were considered sufficiently validated.

Further deterministic gain/plant testing was intentionally stopped.

---

# 8. PPO checkpoint selected for deployment

The successful V3 PPO run was explicitly identified and replayed before deployment.

Run:

```text
2026-09-07_15-57-38_v3_baseline_kp300_kd40_tl45_115
```

Deployment checkpoint:

```text
model_550.pt
```

The checkpoint was tested again in MuJoCo using:

```bash
uv run play Mjlab-Velocity-Flat-Spot-V3   --checkpoint-file "$RUN/model_550.pt"
```

The locomotion was visually confirmed to be excellent.

This checkpoint was therefore frozen as the first V3 sim-to-sim deployment baseline.

---

# 9. V3 policy export

The selected checkpoint was exported to deployment formats.

Exported files:

```text
exported/spot_v3_model_550/
├── policy_jit.pt
├── policy.onnx
└── policy_metadata.json
```

The exporter validated the actor interface as:

```text
input:  (1, 48)
output: (1, 12)
```

It also compared the exported TorchScript model directly with the loaded PPO inference policy.

Result:

```text
max absolute difference = 0.00000000e+00
```

Therefore the TorchScript deployment artifact is numerically identical to the loaded PPO actor for the tested observation.

The ONNX export also succeeded.

---

# 10. V3 actor observation contract

The deployed V3 flat policy has exactly 48 actor observations.

Layout:

```text
0:3    base_lin_vel
3:6    base_ang_vel
6:9    projected_gravity
9:21   joint_pos relative to home
21:33  joint_vel
33:45  previous_action
45:48  command [vx, vy, wz]
```

There is no terrain height scan in the V3 flat policy.

This is a major simplification compared with the old V2 rough-terrain deployment, whose actor expected 235 observations including a 187-value height scan.

The new policy therefore requires only state information already available from Gazebo/ROS.

---

# 11. V3 action contract

The policy outputs 12 normalized joint actions.

The deployment mapping is:

```text
q_ref = q_home + action_scale × action
```

Per-joint-type scales:

```text
HX = 0.30 rad
HY = 0.40 rad
KN = 0.50 rad
```

Expanded scale vector:

```text
0.30, 0.40, 0.50,
0.30, 0.40, 0.50,
0.30, 0.40, 0.50,
0.30, 0.40, 0.50
```

The same joint order is used in training and ROS deployment.

---

# 12. ROS policy package reuse

The existing `spot_policy` ROS 2 package from the older deployment was reused instead of rewritten from scratch.

The package already contained:

```text
observation_monitor.py
policy_dry_run.py
policy_controller.py
```

The old 235-observation metadata and old policy model were replaced with the new V3 export.

The observation monitor was already metadata-driven, so when the V3 metadata removed `height_scan`, the ROS observation builder naturally reduced itself to the new 48-value actor input.

This avoided unnecessary deployment-code churn.

---

# 13. Python environment issue and resolution

When the new V3 TorchScript policy was first launched with:

```bash
ros2 run spot_policy policy_dry_run
```

ROS failed with:

```text
ModuleNotFoundError: No module named 'torch'
```

The reason was not a missing installation in the project.

The existing workspace already had a correct dedicated Python environment:

```text
~/Projects/spot_ros2_ws/.venv
```

Its configuration was:

```text
Python 3.12.3
include-system-site-packages = true
Torch 2.13.0+cpu
rclpy from ROS Jazzy
```

However, `spot_policy` had recently been rebuilt with the system interpreter, causing the generated ROS executable to contain:

```text
#!/usr/bin/python3
```

instead of the workspace virtual environment Python.

This explained why `ros2 run` could not import PyTorch.

The important finding was that no new environment was needed.

The correct deployment environment had already been created during the previous ROS work.

---

# 14. PPO dry-run validation

Before allowing the PPO policy to command the robot, the policy was run in dry-run mode.

The dry-run node:

- reads ROS state
- reconstructs the 48-value observation
- executes the TorchScript policy
- computes joint targets
- does **not** publish joint commands

## Fallen-state dry run

When Spot was lying flat on the ground because no controller reference was active:

```text
base height ≈ 0.102 m
mean absolute PPO action ≈ 0.471
```

The policy produced relatively large and asymmetric actions.

This was correctly interpreted as the PPO reacting to a fallen state rather than as a policy problem.

## Standing-state dry run

Spot was then held at the validated nominal standing reference.

Representative result:

```text
base height:     0.436 m
command:         [0, 0, 0]
observation:     48 values
observation min: -1.000
observation max: +0.262
action min:      -0.047
action max:      +0.262
mean |action|:    0.089
```

Representative proposed targets:

```text
fl_hx  +0.0418
fl_hy  +1.0766
fl_kn  -1.7851

fr_hx  +0.0241
fr_hy  +1.0227
fr_kn  -1.6922

hl_hx  +0.0294
hl_hy  +1.0241
hl_kn  -1.6691

hr_hx  -0.0141
hr_hy  +1.0403
hr_kn  -1.7865
```

The actions were small and stable.

The output converged to a fixed point instead of oscillating.

The dry-run inference counter increased by approximately 50 evaluations per second, confirming the intended 50 Hz PPO update rate.

This validated the ROS observation pipeline and TorchScript inference path.

---

# 15. Live Gazebo PPO deployment

After the dry-run validation, the live `policy_controller` was tested.

The policy node publishes joint-position references to:

```text
/spot_joint_controller/reference
```

using `MultiDOFCommand` with:

```text
values     = q_ref
values_dot = 0
```

A forward velocity command was tested:

```text
vx = 0.5 m/s
vy = 0
wz = 0
```

Result:

```text
Spot walked very nicely in Gazebo.
```

This is the first successful V3 MuJoCo → ROS 2/Gazebo locomotion transfer.

The complete deployed loop is now functional:

```text
/cmd_vel
    ↓
48-value ROS actor observation
    ↓
TorchScript PPO @ 50 Hz
    ↓
12 policy actions
    ↓
q_ref = home + scale × action
    ↓
ros2_control PID @ 2000 Hz
    ↓
Kp = 300, Kd = 40
    ↓
±45 / ±115 Nm effort limits
    ↓
Gazebo physics @ 2000 Hz
```

---

# 16. Important engineering conclusions

## 16.1 The V3 PPO itself was not the source of the Gazebo instability

The fixed nominal-pose test failed before PPO was involved.

The instability therefore belonged to the Gazebo low-level implementation.

## 16.2 Equal gains do not imply equal numerical behavior across simulators

`Kp = 300`, `Kd = 40` behaved differently depending on the simulator's actuator and integration implementation.

A controller equation that looks identical on paper is not necessarily numerically equivalent between:

```text
MuJoCo native implicit actuator
```

and:

```text
Gazebo external effort PID
```

## 16.3 Increasing Gazebo low-level rate solved the transfer problem without changing the approved controller

The successful solution was:

```text
500 Hz → 2000 Hz
```

for Gazebo physics and `ros2_control`.

No new gain tuning was needed.

## 16.4 Torque limits were not the root problem

The same:

```text
±45 Nm hips
±115 Nm knees
```

worked well once the Gazebo update rate was increased.

This is consistent with the earlier MuJoCo A/B experiments showing that the V3 torque limits were not responsible for the bad gait.

## 16.5 The V3 flat deployment is substantially cleaner than the previous rough-policy deployment

The old rough policy required 235 observations including a 187-value terrain height scan.

The new V3 flat deployment requires only:

```text
48 observations
```

with no terrain scan.

This makes the policy much easier to deploy in ROS 2 and much more compatible with realistic onboard state estimation.

## 16.6 Exact policy export parity was achieved

The deployed TorchScript actor matched the loaded PPO actor with:

```text
0.0 maximum absolute error
```

for the verification input.

Therefore any remaining MuJoCo/Gazebo behavior difference should be investigated in observations, coordinate frames, plant dynamics, contact, low-level control, or timing rather than assuming the exported network is different.

---

# 17. Current approved baseline

## MuJoCo / training

```text
Task:
Mjlab-Velocity-Flat-Spot-V3

Checkpoint:
model_550.pt

Run:
2026-09-07_15-57-38_v3_baseline_kp300_kd40_tl45_115

Rigid-body mass:
31.5999994 kg

Physics:
500 Hz

Policy:
50 Hz

Actuator:
BuiltinPositionActuator

Kp:
300

Kd:
40

Hip effort limit:
±45 Nm

Knee effort limit:
±115 Nm
```

## ROS 2 / Gazebo

```text
ROS:
Jazzy

Gazebo:
Harmonic

Hardware interface:
effort

Low-level controller:
pid_controller/PidController

Gazebo physics:
2000 Hz

ros2_control:
2000 Hz

PPO:
50 Hz

Kp:
300

Kd:
40

Hip effort limit:
±45 Nm

Knee effort limit:
±115 Nm

Policy artifact:
policy_jit.pt

Policy input:
48 observations

Policy output:
12 actions
```

---

# 18. What should remain frozen

The following should not be changed without new evidence:

```text
V3 rigid-body inertials
31.5999994 kg mass model
home pose
joint order
action scaling
Kp = 300
Kd = 40
±45 Nm hip limit
±115 Nm knee limit
50 Hz policy rate
MuJoCo 500 Hz training physics
Gazebo 2000 Hz physics/control baseline
V3 reward design
model_550.pt deployment baseline
```

In particular, the previous actuator/gait problem has already been causally resolved.

There is currently no reason to reopen:

- gait reward tuning
- air-time reward tuning
- gain sweeps
- torque-limit sweeps
- actuator-model redesign
- interpolation between policy actions
- artificial delays
- friction/armature additions

unless future deployment evidence specifically points to one of them.

---

# 19. Current project status

```text
V3 rigid-body model
        PASS

V3 MuJoCo deterministic validation
        PASS

V3 PPO locomotion
        PASS

Kd root-cause investigation
        PASS

Gazebo V3 model transfer
        PASS

Gazebo low-level controller
        PASS

500 Hz instability characterization
        PASS

2000 Hz Gazebo baseline
        PASS

stable standing
        PASS

dynamic knee sanity check
        PASS

PPO export
        PASS

TorchScript parity
        PASS

48-observation ROS pipeline
        PASS

PPO dry run
        PASS

live vx = 0.5 m/s Gazebo locomotion
        PASS
```

The first V3 sim-to-sim locomotion transfer is therefore successful.

---

# 20. Recommended next stage

The next work should focus on using the successful deployment rather than reopening the plant.

Recommended progression:

```text
1. Preserve this exact working baseline in Git.

2. Validate the remaining command axes:
   - forward/backward vx
   - lateral vy
   - yaw wz
   - combined commands

3. Connect the PS5 controller to /cmd_vel.

4. Test smooth real-time teleoperation in Gazebo.

5. Only if sim-to-sim behavior exposes a specific mismatch,
   investigate that mismatch directly.

6. After the Gazebo deployment is reliable,
   move toward the real low-level ROS 2 control architecture.
```

The key engineering principle going forward should remain:

```text
Do not change a validated component unless new evidence points to it.
```

The current V3 system is the strongest baseline produced so far in the project.
