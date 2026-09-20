# Spot V3 Gazebo Collapse — Root Cause Analysis and Resolution

## Executive summary

An intermittent catastrophic collapse was observed when deploying the Spot V3 RL locomotion policy in ROS 2 / Gazebo. The robot could walk normally for long periods, including forward motion, backward motion, and pure yaw rotation, but would occasionally **jump violently upward and fall onto its back**, especially during combined translational and yaw commands.

The problem affected both the earlier `model_550` policy and the newer robust `model_600` policy.

A ROS 2 bag captured during a reproducible failure showed that the collapse was **not caused by the policy, command transition, torque limits, or low-level PID controller**. The root cause was a known Gazebo Sim `OdometryPublisher` bug involving **quaternion sign discontinuities**.

At the failure instant, Gazebo published an impossible angular velocity of approximately:

```text
wx ≈  -242.54 rad/s
wy ≈   -87.57 rad/s
wz ≈ -1229.28 rad/s
```

even though the robot had been rotating normally immediately beforehand.

That corrupted angular-velocity observation entered the RL policy. The policy then produced extremely large actions, which generated enormous joint-position references. The `Kp=300`, `Kd=40` low-level controller correctly attempted to follow those references, saturated the available torques, and physically launched the simulated robot.

The causal chain was therefore:

```text
Gazebo quaternion sign discontinuity
        ↓
false odometry angular-velocity spike
        ↓
corrupted RL observation
        ↓
extreme policy action
        ↓
extreme joint-position reference
        ↓
low-level PID torque saturation
        ↓
robot launches and collapses
```

Gazebo Sim 8.15.0 contains the upstream fix for this exact problem. After updating to a runtime containing the fix, the exact previously failing command was left running for approximately **3 minutes without another collapse**.

The locomotion controller and RL policy therefore remain valid and should not be retuned because of this incident.

---

## 1. System context

The failure occurred in the ROS 2 / Gazebo deployment of the Spot V3 locomotion stack.

### Locomotion policy

The deployed policies were:

- earlier deployment: `model_550`
- later robust deployment: `model_600`

Both use the same deployment contract:

- 48 actor observations
- 12 policy actions
- 12 joint-position references
- policy rate: approximately 50 Hz
- same Spot V3 joint order
- same home pose
- same action scales

The policy observation contains:

```text
0:3    base linear velocity
3:6    base angular velocity
6:9    projected gravity
9:21   relative joint positions
21:33  joint velocities
33:45  previous actions
45:48  commanded vx, vy, wz
```

### ROS / Gazebo control architecture

```text
/cmd_vel
   ↓
Spot RL policy @ ~50 Hz
   ↓
q_ref[12]
   ↓
spot_joint_controller
   ↓
PID:
tau = 300 * (q_ref - q) - 40 * qdot
   ↓
effort clamp
hips:  ±45 N·m
knees: ±115 N·m
   ↓
gz_ros2_control
   ↓
Gazebo
```

The Gazebo / ros2_control deployment baseline was:

```text
Gazebo physics:       2000 Hz
controller manager:   2000 Hz
policy:                 ~50 Hz

Kp: 300
Kd: 40

hip effort limit:   ±45 N·m
knee effort limit: ±115 N·m
```

This baseline had already been validated before the intermittent collapse investigation.

---

## 2. Original symptom

The robot behaved normally for extended periods, then suddenly jumped and collapsed.

The failure initially appeared correlated with command changes because it seemed more likely to happen during mixed motion.

A typical observation was:

```text
normal walking
normal walking
normal walking
sudden vertical jump
robot lands / collapses on its back
```

The failure was intermittent rather than immediate.

Importantly, it occurred with both:

```text
model_550
model_600
```

This was an early indication that the new robust policy itself was unlikely to be the root cause.

---

## 3. Reproduction testing

Several simple tests were performed.

### Long forward motion

Approximately:

```text
vx = +0.5 m/s
vy = 0
wz = 0
```

was run for about 40 seconds.

Result:

```text
PASS
No collapse.
```

### Long backward motion

Backward motion was run for approximately 20 seconds.

Result:

```text
PASS
No collapse.
```

### Pure yaw

Pure rotation was tested in both directions for approximately 15 seconds.

Result:

```text
PASS
No collapse.
```

### Combined translation + yaw

A command such as:

```text
vx = +0.3 m/s
vy = 0
wz = +0.5 rad/s
```

was then used.

The exact ROS command was:

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

This command reproducibly caused the robot to collapse after several seconds in some runs.

This made the combined-motion operating condition a useful reproducer.

---

## 4. Checks that ruled out several simpler causes

### 4.1 Duplicate joint-reference publisher

The topic:

```text
/spot_joint_controller/reference
```

had exactly one publisher:

```text
spot_policy_controller
```

and exactly one subscriber:

```text
spot_joint_controller
```

Therefore, two policy nodes were not fighting over the joint targets.

### 4.2 Duplicate `/cmd_vel` publisher

When inspected between tests, `/cmd_vel` had:

```text
Publisher count: 0
Subscription count: 1
```

with the policy controller as the subscriber.

This ruled out an unknown persistent velocity-command source.

### 4.3 Controller state

The active controllers were:

```text
spot_joint_controller
joint_state_broadcaster
```

with both active as expected.

### 4.4 Control configuration

The controller configuration remained the approved deployment baseline:

```text
controller manager rate: 2000 Hz
Kp = 300
Kd = 40
hip clamp = ±45 N·m
knee clamp = ±115 N·m
```

No unexpected controller gain or effort-limit change was found.

---

## 5. Why the console logs were initially insufficient

The policy controller printed diagnostics approximately once per second.

The policy itself was running at approximately 50 Hz.

Therefore, each printed line represented only about:

```text
1 out of every 50 policy updates
```

A single corrupted policy step lasting about 20 ms could therefore occur completely between two printed diagnostics.

This mattered because the actual failure was eventually found to begin with one invalid odometry observation, followed by one extreme policy output.

The normal 1 Hz log was simply too sparse to guarantee that such an event would be visible.

---

## 6. ROS 2 bag capture

A ROS bag was recorded with:

```bash
ros2 bag record \
  /cmd_vel \
  /spot/odom \
  /joint_states \
  /spot_joint_controller/reference
```

The captured bag was:

```text
rosbag2_2026_09_20-14_46_47
```

Storage format:

```text
MCAP
```

### Bag metadata

Total duration:

```text
41.147813538 s
```

Total messages:

```text
86,116
```

Topic counts:

```text
/spot_joint_controller/reference   2,050 messages
/joint_states                     82,010 messages
/spot/odom                         2,056 messages
```

Approximate rates derived from the recording:

```text
policy joint references:
2050 / 41.1478 ≈ 49.82 Hz

Gazebo odometry:
2056 / 41.1478 ≈ 49.97 Hz

joint states:
82010 / 41.1478 ≈ 1993 Hz
```

This also explained a previously observed `~1990 Hz` topic-rate measurement: it corresponded to `/joint_states`, not policy inference or odometry.

The control and observation rates were therefore fundamentally consistent with the intended architecture.

---

## 7. Failure timeline from the captured run

Before the collapse, the robot was executing:

```text
vx = +0.3 m/s
vy = 0
wz = +0.5 rad/s
```

and behaved normally for several seconds.

The body height was around:

```text
0.445–0.450 m
```

and the policy actions remained in a normal range.

Immediately before failure, the joints were also behaving normally.

Then the Gazebo odometry stream generated a physically impossible angular-velocity sample.

Approximately:

```text
wx =  -242.54 rad/s
wy =   -87.57 rad/s
wz = -1229.28 rad/s
```

The magnitude of the angular velocity was roughly:

```text
|ω| ≈ 1256 rad/s
```

This is physically impossible for this robot and many orders of magnitude outside the training distribution.

### Consequence in the policy

The RL policy directly consumes:

```text
base angular velocity
```

as observation dimensions `3:6`.

Therefore the impossible Gazebo value was passed directly into the network.

The policy output, which had previously been of order:

```text
|action| ≈ O(1)
```

became extreme.

The captured failure showed approximately:

```text
minimum action ≈ -68.7
maximum action ≈ +99.4
```

One joint-position reference changed by approximately:

```text
49.7 rad
```

in a single policy update.

Those are not meaningful robot commands. They are a neural-network response to an observation far outside any physically meaningful training distribution.

### Low-level controller reaction

The low-level controller then received the enormous target and evaluated:

```text
tau = 300 * (q_ref - q) - 40 * qdot
```

The requested torque immediately exceeded the configured physical limits.

The controller therefore saturated at:

```text
hips:  ±45 N·m
knees: ±115 N·m
```

This produced very large joint accelerations and joint velocities.

The robot was then launched upward and subsequently collapsed.

The high-gain controller did not originate the error; it faithfully reacted to an invalid reference.

---

## 8. Quaternion discontinuity found in the odometry

The corrupted angular velocity was traced to the orientation representation.

For unit quaternions:

```text
q
```

and:

```text
-q
```

represent exactly the same physical rotation.

A continuous physical orientation can therefore occasionally appear as:

```text
q_previous
-q_current
```

even though the actual orientation changed only slightly.

A correct angular-velocity calculation must account for this quaternion double-cover property.

### What happened in the bag

Around the problematic odometry sample, consecutive quaternions were almost exact negatives.

Their dot product was approximately:

```text
q_previous · q_current ≈ -0.99998
```

For a smoothly varying quaternion sequence we would instead expect the selected representation to remain in the same hemisphere, giving a dot product near:

```text
+1
```

The physical orientation itself did **not** suddenly jump.

Only its quaternion sign representation changed.

An odometry implementation that directly differences these two quaternion representations can interpret the sign change as an almost complete rotation over one tiny simulation step.

That produces a false angular velocity of enormous magnitude.

---

## 9. Upstream Gazebo bug

This behavior matches a documented Gazebo Sim bug:

```text
Gazebo Sim issue #3830
"OdometryPublisher - published Pose Orientation Quaternion is non-continuous over time"
```

The issue describes the same mechanism:

1. `q` and `-q` represent the same orientation.
2. The published pose quaternion can switch sign.
3. `OdometryPublisher` differences consecutive orientations.
4. Without enforcing quaternion sign continuity, the sign flip is interpreted as a large rotation.
5. This creates very large false angular-velocity spikes.

The upstream fix was implemented in:

```text
Gazebo Sim PR #3834
"Fix quaternion sign-flip causing angular velocity spikes"
```

The essential fix is conceptually:

```cpp
currentRot = currentPose.Rot();

if (currentRot.Dot(lastPose.Rot()) < 0)
{
    currentRot = -currentRot;
}

rotationDiff =
    currentRot * lastPose.Rot().Inverse();
```

The dot-product test forces the current quaternion into the same quaternion hemisphere as the previous one before calculating the relative rotation.

---

## 10. Why the bug appeared intermittent

The failure was not guaranteed on every rotation.

Gazebo's odometry system internally evaluates state at a much higher simulation rate than its configured publication rate.

In this deployment:

```text
physics:        2000 Hz
odom publish:     50 Hz
```

The odometry implementation also applies short rolling averages to its velocity estimates.

Therefore, whether a one-timestep quaternion sign discontinuity contaminates an externally published `/spot/odom` message depends on timing.

The bag showed several quaternion sign changes during the run, but only the problematic alignment generated the catastrophic published angular-velocity observation.

This explains the apparently random behavior:

```text
same command
same controller
same policy
same environment

sometimes:
    no problem

sometimes:
    one bad odometry frame
    → catastrophic policy response
```

It also explains why the robot could rotate successfully for many seconds or multiple turns before suddenly failing.

---

## 11. Gazebo version investigation

The first version check gave:

```bash
gz sim --versions
```

result:

```text
8.11.0
```

However the system package database showed:

```text
gz-sim8-cli             8.14.0-1~noble
libgz-sim8              8.14.0-1~noble
libgz-sim8-dev          8.14.0-1~noble
libgz-sim8-plugins      8.14.0-1~noble
python3-gz-sim8         8.14.0-1~noble
```

and:

```text
apt candidate for libgz-sim8:
8.15.0-1~noble
```

At the same time, ROS Jazzy had:

```text
ros-jazzy-gz-sim-vendor
0.0.10-1noble...
```

which provided / selected a Gazebo Sim 8.11.0 runtime in the sourced ROS environment.

This explained why the executable actually being used still reported:

```text
8.11.0
```

despite newer system Gazebo packages being installed.

### Relevant Gazebo release

The Gazebo Sim 8.x changelog for:

```text
Gazebo Sim 8.15.0
2026-08-11
```

explicitly contains:

```text
Fix quaternion sign-flip causing angular velocity spikes (#3834)
```

Therefore 8.15.0 is the relevant fixed Harmonic release.

---

## 12. Resolution

The runtime was updated to a Gazebo version containing the upstream quaternion-continuity fix.

No locomotion policy retraining was required.

No changes were made to:

```text
Kp
Kd
torque limits
policy action scaling
reward function
PPO architecture
joint model
Gazebo physics rate
ros2_control rate
```

The root cause was corrected at the odometry source instead.

---

## 13. Post-fix validation

After the Gazebo update, the exact reproducer was run again:

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

Before the fix:

```text
collapse could occur after only a few seconds
```

After the fix:

```text
same command ran for approximately 3 minutes
with no collapse
```

This is strong operational confirmation that the failure was caused by the Gazebo odometry bug.

---

## 14. Components cleared by the investigation

The following components should **not** be treated as root causes of this incident.

### `model_600`

The policy behaved normally until receiving a physically impossible observation.

The extreme output occurred only after the odometry corruption.

Therefore the robust policy should not be retrained because of this failure.

### `model_550`

The same historical failure had also been observed with `model_550`.

This is consistent with a shared simulator-observation problem rather than a specific trained checkpoint.

### Low-level `300 / 40` controller

The low-level controller responded to an enormous invalid joint reference.

Its saturation was a downstream consequence.

There is no evidence from this incident requiring the already validated gains to be reopened.

### Torque limits

The torque clamps prevented even larger efforts.

They did not create the original bad command.

### Command transitions

The failure could occur after the combined command had been held constant for several seconds.

Therefore the primary cause was not an instantaneous `/cmd_vel` transition.

### Duplicate ROS publishers

The relevant command/reference topics had the expected publisher topology.

No competing joint-command publisher was found.

---

## 15. Important engineering lesson: validate observations, not only outputs

The failure illustrates a critical deployment principle for learned controllers.

The policy itself may be correct, while a single corrupted observation can still generate an unsafe output.

The observation chain is therefore part of the controller:

```text
sensor / simulator
      ↓
state estimate
      ↓
observation construction
      ↓
policy
      ↓
actuator reference
```

A deployment test that verifies only:

```text
policy output is finite
```

is insufficient.

The inputs should also satisfy basic physical sanity constraints.

---

## 16. Recommended defense-in-depth protection

The upstream Gazebo fix is the real solution.

However, the ROS deployment should eventually include a small observation-sanity layer so that one corrupted sensor or estimator packet cannot produce a catastrophic joint command.

This should not alter normal policy behavior.

Possible checks include:

```text
all observation values finite

reasonable base angular velocity
reasonable base linear velocity

reasonable joint velocities

optional maximum per-step action / q_ref jump
```

For example, a deployment guard could reject a frame if:

```text
|ω_body| > 10–20 rad/s
```

because such a value is already far outside any normal Spot locomotion state.

The exact bound should be selected from actual training / validation data rather than arbitrarily.

On detecting an invalid observation, a safe behavior would be:

```text
do not run the policy on that frame
hold the previous valid q_ref
log the rejected sample
continue when valid observations resume
```

This would have prevented the observed collapse even with the Gazebo bug present.

This guard should be considered **defense in depth**, not a substitute for correct odometry.

---

## 17. Recommended regression test

This failure mode should be preserved as a regression test.

A simple deployment test is:

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

Run for at least:

```text
3 minutes
```

while monitoring:

```text
/spot/odom
/spot_joint_controller/reference
/joint_states
```

Acceptance criteria:

```text
no collapse
no abnormal vertical launch
no impossible odometry angular-velocity spike
no extreme one-frame q_ref discontinuity
```

For automated checking, record:

```bash
ros2 bag record \
  /cmd_vel \
  /spot/odom \
  /joint_states \
  /spot_joint_controller/reference
```

and verify:

```text
max physically reasonable |ω|
max q_ref step
max qdot
body-height range
```

This provides a reproducible regression test independent of visual inspection.

---

## 18. Final status

### Root cause

```text
CONFIRMED
```

Gazebo Sim `OdometryPublisher` quaternion sign discontinuity caused a false angular-velocity spike.

### Policy fault

```text
NO
```

The policy reacted to an impossible observation.

### Low-level controller fault

```text
NO
```

The controller reacted to an impossible policy-generated reference.

### Model-specific fault

```text
NO EVIDENCE
```

The same symptom occurred with multiple checkpoints.

### Fix

```text
Use Gazebo Sim 8.15.0 or another runtime containing the
OdometryPublisher quaternion-continuity fix.
```

### Validation

```text
Previously failing:
vx = +0.3 m/s
wz = +0.5 rad/s

Post-fix runtime:
~3 minutes continuous execution
No collapse observed.
```

### Deployment decision

The Spot V3 locomotion baseline can remain frozen:

```text
model_600
50 Hz policy
2000 Hz Gazebo physics
2000 Hz ros2_control
Kp = 300
Kd = 40
hips  ±45 N·m
knees ±115 N·m
```

Future work can proceed to LiDAR, SLAM, and navigation without reopening the locomotion controller because of this issue.

---

## 19. Reference material

Upstream Gazebo references:

- `gazebosim/gz-sim` issue `#3830`  
  **OdometryPublisher - published Pose Orientation Quaternion is non-continuous over time**

- `gazebosim/gz-sim` pull request `#3834`  
  **Fix quaternion sign-flip causing angular velocity spikes**

- Harmonic / `gz-sim8` backport pull request `#3836`

- Gazebo Sim `8.15.0` changelog entry:  
  **Fix quaternion sign-flip causing angular velocity spikes (#3834)**

Captured project evidence:

```text
rosbag2_2026_09_20-14_46_47
```

Topics captured:

```text
/cmd_vel
/spot/odom
/joint_states
/spot_joint_controller/reference
```
