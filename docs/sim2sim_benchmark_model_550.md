# MuJoCo vs Gazebo velocity tracking (baseline `model_550`)

> This benchmark was run on the earlier non-robust baseline `model_550`, not on the deployed robust `model_600`.
> Back to the [project README](../README.md).

The only quantitative MuJoCo–Gazebo comparison was run on the earlier non-robust baseline `model_550` (task `Mjlab-Velocity-Flat-Spot-V3`, Kp=300, Kd=40), using the same scripted 59 s command sequence in both simulators (stand, ±0.5 m/s forward/backward, ±0.3 m/s lateral, ±0.5 rad/s yaw, one combined command).
Data: [`evaluation_runs/spot_v3_gazebo_eval_20260910_101049/`](../evaluation_runs/spot_v3_gazebo_eval_20260910_101049) (a second Gazebo run, `…_101248`, agrees closely except yaw, where the yaw-left error was 0.300 rad/s) and, in the training repo, `benchmark_runs/mujoco/spot_v3_mujoco_eval_20260910_111210/`.

Metric: for each simulator separately, the mean of |measured base velocity − commanded velocity| over the samples of a phase after its first 1.0 s. It is a **tracking error against the command**, not a direct MuJoCo-vs-Gazebo difference, and it includes oscillation as well as steady bias. Each row reports the error on the commanded axis (Gazebo run `…_101049`; one run per simulator, no confidence intervals):

| Phase | Command | MuJoCo error | Gazebo error |
|---|---|---|---|
| Forward | vx = 0.5 m/s | 0.028 m/s | 0.017 m/s |
| Backward | vx = −0.5 m/s | 0.024 m/s | 0.016 m/s |
| Left | vy = 0.3 m/s | 0.052 m/s | 0.024 m/s |
| Yaw left | wz = 0.5 rad/s | 0.209 rad/s | 0.220 rad/s |
| Combined | 0.4 m/s, 0.2 m/s, 0.3 rad/s | 0.031 / 0.021 / 0.030 | 0.017 / 0.021 / 0.062 |

(Combined shows vx / vy / wz error. The summaries record no joint-torque saturation in these phases.)

**Reading the results.** Linear-velocity tracking is within about 0.02–0.05 m/s in both simulators. Yaw tracking is much worse: roughly 0.2 rad/s of error on a 0.5 rad/s command (about 40 % of the command) in both simulators, and 0.30 rad/s in the second Gazebo run. Yaw tracking is therefore a limitation of this policy, present in MuJoCo as well as Gazebo, and not a sim-to-sim transfer effect. The comparison supports "similar tracking behaviour in the two simulators", not equivalence of the dynamics.

**Not yet measured:** an equivalent benchmark for the deployed `model_600`. The `model_600` behaviour in Gazebo is documented qualitatively only.
