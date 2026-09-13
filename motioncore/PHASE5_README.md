# MotionCore Phase 5A: count-based encoder feedback

**Execution status: prepared and syntax-checked; MATLAB/Simulink validation is pending.**
The implementation is based on your corrected GitHub Phase 4 commit
`d5c44240e227d2556099288f473bf3720b23027b` ("Validate MotionCore Phase 4 averaged
PWM driver"). Its saved Phase 4 validation flag and four regression rows pass;
the largest saved Phase 3/4 RPM discrepancy is about `4.55e-13 RPM`.
The corrected model's root-level connections and block interfaces have been
inspected directly. MATLAB/Simulink is unavailable in the preparation runtime.
No Phase 5 SLX or Simulink PASS result is claimed. The runner below builds and
validates against your saved, corrected Phase 4 model on your MATLAB installation.

All changes in this package are new Phase 5 files. Do not replace your corrected
Phase 1--4 scripts with older files from the preceding conversation.

## Run

Put the Phase 5 `.m` files beside the validated models and existing scripts:
`MotionCore.slx`, `MotionCore_Phase3.slx`, `MotionCore_Phase4.slx`,
`check_motioncore.m`, `check_motioncore_phase3.m`, `check_motioncore_phase4.m`,
their reference/metrics helpers, and `motioncore_plant_signature.m`.
Save any open model edits before running.

```matlab
summary = run_motioncore_phase5;
```

Or give the absolute folder holding the frozen models:

```matlab
summary = run_motioncore_phase5(pwd);
```

The runner first checks frozen Phase 1--4 behavior, including nominal speed
tests and the existing anti-windup stress case. It copies the **whole Phase 4
SLX**, including root-level branches, to `MotionCore_Phase5.slx`. It tests that
untouched copy at all four speeds before adding the encoder. Missing or
disconnected logs fail before performance checks.

It then changes only the source feeding `Feedback`, renames that subsystem's
input to `Encoder_RPM`, adds `Encoder`, `RPM_Estimator` and validation logs, and
checks the final topology. `Digital_PID`, `Voltage_Command`, `PWM_Driver`,
`Reference`, and `DC_Motor_Plant` are protected by structural comparisons.

Original model/script bytes are checked before and after the run. Previous
runners are not called because they can rebuild the frozen models. An existing
Phase 5 model is backed up under `phase5_backups` before rebuilding. Each run
uses a new subfolder of `results_phase5`; previous results are retained.

To include the separate low-speed and encoder-resolution experiments:

```matlab
summary = run_motioncore_phase5(pwd, true);
```

These simulations use temporary model-workspace overrides, without saving the
experimental encoder settings. A coarse-encoder experiment may fail baseline
tracking criteria: that outcome is recorded, and does not redefine the primary
Phase 5 acceptance criteria. Mandatory baseline failures stop the runner.

Required products: MATLAB and Simulink. No Control System Toolbox, Simscape,
Fixed-Point Designer, or HDL Coder is needed. The reference function itself uses
MATLAB `expm` and tables, with no Simulink dependency.

## Encoder definition and timing

- `ENCODER_PPR = 1024`: cycles per mechanical revolution on one conceptual channel.
- `decode_multiplier = 4`: quadrature x4 interpretation, without A/B waveforms yet.
- `ENCODER_CPR = 4096`: count increments per mechanical revolution; used internally.
- `Tenc = 0.001 s`: nonoverlapping count-difference window and estimator update period.
- `Ts = 0.001 s`: preserved controller period; `fpwm = 20 kHz`, `Vdc = 12 V` remain unchanged.

The physical encoder front end is simulated using

\[
\theta(t)=\theta_0+\int_0^t\omega(\tau)\,d\tau,\qquad
C[n]=\left\lfloor\frac{CPR\,\theta(nT_{enc})}{2\pi}\right\rfloor.
\]

Angle is sampled **before** the floor operation. This models the snapshot of
an ideal cumulative counter without scheduling every edge in the simulation.
Counts are integer-valued doubles in this behavioral model, exact below
`2^53`; counter width, wrapping, missing edges, and clock-domain crossing are
not yet modeled. Floor is defined consistently for signed angle as well.

At each estimator hit:

\[
\Delta C[n]=C[n]-C[n-1],\qquad
\widehat{RPM}[n]=\Delta C[n]\frac{60}{CPR\,T_{enc}}.
\]

The previous-count register is initialized to the count at `theta_initial_rad`,
so the first delta and estimated speed are zero for the validated rest startup.
The new estimate is available to the PI at that sample hit, then the PI state
updates using the existing Forward-Euler recurrence. No extra computational
delay or smoothing filter is added.

`Tenc` is configurable as an integer multiple of `Ts`, with `Tenc >= Ts`.
For a longer window, the estimate is held until the next estimator hit and the
existing feedback ZOH continues feeding the PI at `Ts`. This restricted rate
relationship is explicit; arbitrary asynchronous rates are not implemented.

The estimator is based on preceding-window motion. It has approximately
`Tenc/2` measurement age at the update under smooth motion, plus additional age
while the result is held. A longer window improves RPM resolution in direct
proportion to its length, while adding lag. Hardware pipeline latency is zero
in this reference and needs explicit treatment in a later phase.

## Resolution and engineering checks

\[
q_{RPM}=\frac{60}{4096(0.001)}=14.6484375\ \text{RPM/count}.
\]

This is the spacing between adjacent possible speed estimates. At 1000 RPM,
the mean is 68.2667 counts/window, so 68- and 69-count windows produce 996.09375
and 1010.7421875 RPM. It is normal for measured RPM to jump between them even
when the actual shaft speed is nearly constant.

At 50 RPM, only 3.4133 counts/window are available: estimates commonly alternate
between 43.9453125 and 58.59375 RPM. The absolute step size is unchanged, but is
a much larger fraction of the requested speed.

The checker distinguishes instantaneous error from pure count quantisation:

\[
\overline{RPM}[n]=\frac{\theta(nT_{enc})-\theta((n-1)T_{enc})}{T_{enc}}\frac{60}{2\pi},
\qquad |\widehat{RPM}[n]-\overline{RPM}[n]|<q_{RPM}.
\]

The two endpoint floor errors differ by less than one count. Numerical boundary
tolerance is allowed only near an exact count boundary. Instantaneous error
also includes the difference between window-average and end-of-window speed;
it can exceed one RPM/count during acceleration without indicating bad scaling.

The main checker verifies integer counts, angle/count consistency, count-delta
arithmetic, held estimates, count-only feedback, reference and PI algebra,
anti-windup recurrence, PWM identities, exact observed-input ZOH propagation of
current/speed/angle, current bounds, and physical energy conservation. The
runner independently traces feedback wiring and checks protected subsystems.

Finite-duration acceptance criteria are explicit: absolute mean true-speed
error <=1 RPM, maximum tail error <=3 RPM, settling <=0.5 s, overshoot <=5%,
and physical peak current <=1.02*Vdc/Ra. A further comparison flags more than
0.25 A peak-current growth over the corresponding Phase 4 run. These are
engineering simulation gates, not a proof of stability for every operating
point or a hardware current rating.

Means and RMS metrics use uniform controller samples over the last 0.2 s for
steady-state quantities; rise/settling and peak current use dense solver logs.
Estimator-event tables are saved separately for configurable `Tenc`.

## Independent numerical results (not Simulink output)

`verify_motioncore_phase5.py` reads the motor and PI parameters from your saved
`results_phase3/phase3_validation.mat`. It compares exact augmented ZOH matrix
propagation with independently integrated, event-separated SciPy ODE solutions.
The maximum true-speed difference across the four tests was below `1.8e-10 RPM`.
MATLAB syntax parsing passed for all nine Phase 5 `.m` files.

| Target RPM | Rise (s) | Settling (s) | Mean speed error (RPM) | True-speed tail ripple, p-p (RPM) | Encoder tail ripple, p-p (RPM) |
|---:|---:|---:|---:|---:|---:|
| 500 | 0.1418 | 0.252 | -0.00033 | 0.1415 | 14.6484 |
| 1000 | 0.1418 | 0.253 | 0.000057 | 0.1139 | 14.6484 |
| 1500 | 0.1418 | 0.253 | 0.00325 | 0.0940 | 14.6484 |
| 2000 | 0.1440 | 0.256 | 0.00327 | 0.0994 | 14.6484 |

The 2000 RPM test reaches duty 1 and spends about 42 ms in voltage saturation.
The unreachable 3000-to-1000 RPM stress case recovers with the original `Kb=20`.
The simulated primary configuration does not currently justify PI retuning.

Relative to the independent ideal-feedback Phase 4 equations, maximum transient
true-speed differences are about 1.50, 2.94, 4.35 and 2.34 RPM respectively.
These differences are deliberate measurement effects, not numerical-regression
failures. The saved MATLAB runner performs the actual frozen-model comparisons.

The optional 128-PPR/x4, 50-RPM reference case demonstrates why resolution
matters: its 117.1875 RPM/count estimate is very coarse, and the 0--12 V
controller shows about +8.12 RPM mean true-speed bias over the observed tail.
That case does not settle into the requested band during the run. It is
recorded as an experiment limitation. The 1024-PPR baseline is not changed.

To reproduce the independent results (Python is optional):

```sh
python verify_motioncore_phase5.py --output phase5_reference_results
```

The JSON includes the parameter-source checksum and labels the result
`INDEPENDENT_NUMERICAL_REFERENCE_PASS_NOT_SIMULINK`. CSV and PNG reference
artifacts are also included.

## Files and output

`motioncore_phase5_init.m` holds the primary settings;
`motioncore_phase5_parameters.m` recomputes dependent CPR/scaling for experiments.
`build_motioncore_phase5.m` has explicit copy and encoder stages.
`motioncore_phase5_structure.m` checks protected subsystems and signal sources.
`motioncore_phase5_logs.m` requires populated timeseries and preserves event timing.
`motioncore_phase5_reference.m` provides the independent MATLAB sampled equations.
`motioncore_phase5_metrics.m` computes control/measurement metrics.
`check_motioncore_phase5.m` checks the physical and digital identities.
`run_motioncore_phase5.m` orchestrates all gates and saves results.

Each main test saves dense CSV, estimator-event CSV, full MAT data, checks, and
one six-panel plot. Tables include `rpm`/`true_rpm`, `measured_rpm`/`feedback_rpm`,
and `encoder_rpm` explicitly. True RPM only supplies validation/plotting and the
physical encoder simulation through its underlying angular velocity.

## FPGA mapping and next step

The eventual FPGA estimator needs a cumulative edge counter, an atomic count
snapshot, a previous-count register, subtraction and a constant scale multiply.
The continuous angle integrator and floor operation belong to the simulation's
encoder stimulus model; physical encoder edges replace them in hardware.

At 2000 RPM and 4096 CPR, the hardware must capture about 136,533 count events/s.
The 1 kHz number is the speed-estimation update rate, not an A/B sampling clock.
No edge loss is modeled in Phase 5A.

After the actual MATLAB runner prints `PHASE 5A PASS`, review the measured
metrics/plots. The next optional encoder step is an A/B behavioral generator
and decoder comparison. At low speed, consider a longer count window, moving
average (with explicit delay), pulse-period/reciprocal measurement, or a hybrid
estimator as separate experiments. No such filter, detailed A/B model,
fixed-point PI, HDL, or RTL is included here.

Official API references checked during preparation:
- https://www.mathworks.com/help/simulink/slref/roundingfunction.html
- https://www.mathworks.com/help/simulink/slref/simulink.simulationinput.setvariable.html

If MATLAB fails, retain the generated partial Phase 5 model and send the first
full error/stack and printed check results. Do not rebuild the golden models
to work around a Phase 5 integration error.
