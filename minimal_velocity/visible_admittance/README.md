# Visible-admittance velocity workbench | R2024b

This copy starts from the current `minimal_velocity/robot_ver2_velocity_minimal.slx`. The current baseline and its previous raw results remain in place.

## Run and inspect

From the project root, run this in MATLAB.

```matlab
addpath(fullfile(pwd,'minimal_velocity','visible_admittance'))
mdl = build_visible_admittance;
open_system(mdl)
```

1. Double-click **User Applied Wrench**. Set **Fx, Fy, Fz** in N and **Mx, My, Mz** in N·m. The order is `[Fx Fy Fz Mx My Mz]`. All six components are resolved in world/base axes and applied **at the TCP**. The same wrench branches to **RobotPlant** and **SensorReference**. Only the measurement branch adds sensor effects.
2. Set **Push start** and **Push release** in seconds. Profile **0** gives the existing sine-squared pulse with its peak at the midpoint. Profile **1** applies the selected vector from start up to release. Set the model Stop Time beyond release to observe the response. The default is +0.18 N in X with the pulse from 0.10 to 0.52 s.
3. Admittance is selected by default. **UseStandaloneVelocity** reads `velocityCase.standalone` from **Model Explorer → Model Workspace**. Set it to `false` for admittance and `true` to use the root **JointVelocityCommand** in J1–J6 rad/s. Equivalently, use the commands below. Both mapper enable and the existing command selector use this single setting.
4. Press Run. Open **Response Scopes** and double-click **Wrench Applied and EMA**, **Joint Velocity Command and Measured**, or **Joint Displacement from Park deg**. The displacement scope shows `(q - initialQ)*180/pi` for the measured joints. For a spatial view, open the model's Mechanics Explorer after a run.

```matlab
mw = get_param(mdl,'ModelWorkspace');
mw.assignin('velocityCase',struct('standalone',false));
set_param([mdl '/User Applied Wrench'],'Fx','0.12','pushStart','0.10', ...
    'pushRelease','0.44','holdProfile','1')
set_param(mdl,'StopTime','0.92')
```

## What changed

| Part | Behaviour |
|---|---|
| User Applied Wrench | Replaces the old scripted source. This is the sole physical load source. It uses standard source, gating and math blocks at 2 ms. |
| Cartesian Admittance | The **Previous wrench** and **Previous limited twist** Unit Delays start at six zeros. The visible blocks evaluate the mass–damper Tustin equation, bound the change in twist, then bound total speed. The final limited twist feeds back into the next tick. |
| Cartesian-to-Joint Mapper | The mapper consumes the new limited twist **at the current tick** and publishes joint velocity at the next, as in the baseline. `visible_joint_mapper.m` retains the compact-park, measured-state and proposed-reference guards. The Win64 gateway calls the original vendor `cartesian_joint_mapper_step` and original robot_ver2 C Jacobian. It retains the mapper's position integrator and limits. |
| Response Scopes | Display-only taps. The original logging remains, including published TCP twist (one tick behind the candidate, like the original diagnostic). |
| Preserved | SensorReference/EMA, the existing velocity selection logic, VelocityServo, its gravity feedforward, torque-driven RobotPlant and the original solver settings. The unused parallel laboratory admittance comparison and old combined block were removed from this copy. |

At each 2 ms tick, component by component:

```text
ideal = ((2*M - D*dt)*Vprev + dt*(W + Wprev)) / (2*M + D*dt)
delta = clamp(ideal - Vprev, -accelerationLimit*dt, +accelerationLimit*dt)
V = clamp(Vprev + delta, -speedLimit, +speedLimit)
store V and W for next tick
```

The original virtual mass/inertia is [2, 2, 2, 0.2, 0.2, 0.2]. The original damping is [20, 20, 20, 2, 2, 2]. The linear units are kg and N·s/m, with angular units kg·m² and N·m·s/rad. Stiffness is zero. Velocity caps are 0.05 m/s per linear component and 0.15 rad/s per angular component. Acceleration caps are 0.20 m/s² and 0.50 rad/s² respectively. The twist is in base axes at the TCP, ordered linear XYZ then angular XYZ. The unchanged plant solver is fixed-step ode4 at 0.5 ms. Logs can observe intermediate block calculations and EMA at solver steps. State storage and mapper consumption are both on the 2 ms clock, so the controller comparison uses that clock.

The model uses the supplied mapper-only MEX and its existing project dependencies. The original combined MEX is used only as the validation oracle. WSL was used to build the new binary and can render the batch plots, but Simulink runs the mapper in-process on Win64. The actuator is still the provisional joint-side PI. This work adds no outer position loop.

## Validation

```matlab
folder = run_visible_admittance;
```

The runner saves raw MAT traces, configuration, predeclared criteria and separate controller/plant outcomes under `results`. Optional offline plot generation from PowerShell at project root:

```powershell
wsl.exe --exec python3 minimal_velocity/visible_admittance/plot_visible.py minimal_velocity/visible_admittance/results/<timestamp>
```

Plant response includes the physical load as well as the torque response. A controller parity result establishes numerical equivalence on the tested inputs. The measured traces describe the simulated robot with the provisional actuator.

The original live simulation batch, including its first score and the three isolated fixture errors, is preserved in `results/20261125_162254_382`. The corrected disconnected fixture runs are in `results/20261125_162423_607`. Open `results/20261125_163118_715/index.html` for the combined result browser. The live model simulations were run once. Offline scoring of the saved live traces uses the actual 2 ms mapper clock. The standalone test checks what the selector received under the existing zero-order hold. On 20 ticks the supplied timestamp rounded slightly after the solver tick, so the held value was the preceding sample. The largest difference from the nominal command tick was 0.000571 rad/s. No runtime timing policy was changed.

See `results/mechanical_xml_audit.txt` for the comparison of the preserved RobotPlant and its child subsystems. The copy's save reordered identical instrumentation metadata entries. The mechanical XML and instrumentation values otherwise agree.
