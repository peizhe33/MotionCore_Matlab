# MotionCore — Phase 1 and Phase 2

An equation-based, open-loop permanent-magnet brushed DC motor plant.
No PID, PWM, encoder, switching driver, or fixed-point logic is implemented.

## Run it

1. Extract this package and set MATLAB's **Current Folder** to `motioncore`.
2. Run `run_motioncore` in the Command Window.
3. The builder creates, compiles, saves, and opens `MotionCore.slx`.
4. The runner simulates, prints PASS/FAIL/SKIP checks, plots six panels, and
   saves CSV, MAT, PNG and MATLAB FIG output under `results/`.

```matlab
run_motioncore
```

The SLX is generated on your machine; it is not included in this package.
Requires **MATLAB and Simulink**. Simscape, Simscape Electrical, Control System
Toolbox, DSP System Toolbox and HDL Coder are not required. Scripts use
traditional character-vector name/value pairs and standard blocks. MATLAB
release compatibility has not been runtime-tested here.

| File | Purpose |
| --- | --- |
| `motioncore_init.m` | Editable SI parameters, state-space matrices, transfer-function coefficients and equilibrium predictions |
| `build_motioncore.m` | Programmatically constructs the open-loop Simulink model |
| `run_motioncore.m` | Builds, simulates, checks and plots |
| `motioncore_reference.m` | Exact piecewise-constant-input solution using base MATLAB `expm` |
| `check_motioncore.m` | Engineering checks on the actual logged Simulink signals |
| `expected_response.png` | Independently calculated reference waveforms; not a Simulink screenshot |
| `reference_validation.json` | Numerical results and explicit validation limits |

Edit parameters in `motioncore_init.m`, then rerun `run_motioncore`. The builder
stores a parameter snapshot in the model workspace, so the generated SLX can
also simulate independently. Editing the init script alone does not update an
already-open SLX: rerun the builder. Existing saved SLX files are copied to
`backups/` before rebuilding. Unsaved model changes produce an error rather
than being discarded. For manual block edits, save and simulate the model
directly; the runner deliberately rebuilds from MATLAB source.

## Phase 1: mathematical model

Assume a rigid shaft, constant permanent-magnet flux, linear inductance,
constant resistance, and viscous friction. Positive load torque opposes the
positive shaft direction. Electrical and mechanical states are continuous.

\[
\dot i=\frac{V_a-R_ai-K_e\omega}{L_a},\qquad
\dot\omega=\frac{K_ti-b\omega-T_L}{J}.
\]

With zero initial conditions, Laplace transformation gives

\[
(L_as+R_a)I+K_e\Omega=V_a,\qquad
(Js+b)\Omega=K_tI-T_L.
\]

Eliminate current by multiplying the mechanical equation by \(L_as+R_a\)
and substituting the electrical equation:

\[
\underbrace{[(L_as+R_a)(Js+b)+K_tK_e]}_{D(s)}\Omega
=K_tV_a-(L_as+R_a)T_L.
\]

Therefore

\[
\left.\frac{\Omega(s)}{V_a(s)}\right|_{T_L=0}
=\frac{K_t}{L_aJs^2+(L_ab+R_aJ)s+R_ab+K_tK_e},
\]

\[
\left.\frac{\Omega(s)}{T_L(s)}\right|_{V_a=0}
=-\frac{L_as+R_a}{D(s)}.
\]

The second transfer function is the signed load-to-speed response; it is also
the small-signal disturbance response about an operating point. Current is

\[
I(s)=\frac{(Js+b)V_a(s)+K_eT_L(s)}{D(s)}.
\]

The speed conversion is

\[
n_{\rm RPM}=\omega\frac{60}{2\pi},\qquad
\omega=n_{\rm RPM}\frac{2\pi}{60}.
\]

For \(x=[i\;\omega]^T\), \(u=[V_a\;T_L]^T\), the state-space model is

\[
\dot x=Ax+Bu,\quad
A=\begin{bmatrix}-R_a/L_a&-K_e/L_a\\K_t/J&-b/J\end{bmatrix},\quad
B=\begin{bmatrix}1/L_a&0\\0&-1/J\end{bmatrix}.
\]

For state outputs \(y=[i\;\omega]^T\), \(C=I_2,D=0\). Additional outputs
are algebraic: \(T_e=K_ti\), \(e_b=K_e\omega\), and RPM as above. Voltage
and load are logged directly from their input sources.

## Illustrative motor parameters

These are a coherent teaching example for a small 12 V motor and attached
inertia, not specifications or identified values of a particular motor.

| Parameter | Value | Unit |
| --- | ---: | --- |
| Armature resistance, Ra | 2 | ohm |
| Armature inductance, La | 0.002 | H |
| Back-EMF constant, Ke | 0.05 | V/(rad/s) |
| Torque constant, Kt | 0.05 | N m/A |
| Total shaft-referred inertia, J | 0.0001 | kg m² |
| Viscous friction coefficient, b | 0.0001 | N m/(rad/s) |
| Armature voltage | 0 → 12 at 0.05 s | V |
| Constant external load | 0 | N m |
| Initial current and speed | 0 | A, rad/s |
| Stop time | 0.8 | s |

Kt and Ke are numerically equal in these consistent SI units for an ideal
reciprocal motor. A datasheet Ke in V/krpm or a speed constant Kv in rpm/V
must be converted before use.

The numerical model is

\[
A=\begin{bmatrix}-1000&-25\\500&-1\end{bmatrix},\quad
B=\begin{bmatrix}500&0\\0&-10000\end{bmatrix},
\]

\[
\frac{\Omega}{V_a}=
\frac{0.05}{2\times10^{-7}s^2+2.002\times10^{-4}s+0.0027}.
\]

Poles are approximately -987.327 and -13.6733 s⁻¹, giving decay time constants
of 1.013 ms and 73.14 ms. The uncoupled electrical time constant La/Ra is
1 ms; J/b = 1 s is **not** the observed speed time constant because back EMF
provides electromechanical damping under voltage drive.

## Phase 2: model implementation

Top level contains `Armature_Voltage_Step`, `Load_Torque`, `DC_Motor_Plant`
and `Logging`. Double-click the plant to see:

- Electrical balance Sum (+--), Gain 1/La, and Integrator for current.
- Ra feedback voltage and Ke feedback voltage subtracted at that Sum.
- Kt gain converting current to electromagnetic torque.
- Mechanical balance Sum (+--), Gain 1/J, and Integrator for shaft speed.
- Viscous-friction torque and external load torque subtracted at that Sum.
- RPM conversion and five named plant output ports.

The two integrators break the feedback paths, so this architecture has no
algebraic loop. A constant configurable load input is exposed for plant tests;
load-step disturbances remain a later phase.

The solver is variable-step `ode45`, with RelTol 1e-7, AbsTol 1e-9 and maximum
step 50 microseconds. That maximum step resolves the 1 ms electrical transient
with at least about 20 steps. It is a **solver setting**, not a controller
sampling period. Reassess it after changing parameters. Output logging uses
the solver's actual time vector and seven `To Workspace` timeseries sinks.

## Expected waveforms and numerical targets

At equilibrium,

\[
\omega_{ss}=\frac{K_tV_a-R_aT_L}{R_ab+K_tK_e},\qquad
i_{ss}=\frac{bV_a+K_eT_L}{R_ab+K_tK_e}.
\]

| Quantity | Default prediction |
| --- | ---: |
| Steady shaft speed | 222.222 rad/s = 2,122.066 RPM |
| Steady current | 0.444444 A |
| Steady electromagnetic torque | 0.0222222 N m |
| Steady back EMF | 11.111111 V |
| Startup current peak | approximately 5.743 A |
| Time of current peak | approximately 0.05445 s (4.45 ms after voltage step) |
| Stall current V/R (locked rotor, after electrical transient) | 6 A |
| Stall electromagnetic torque Kt*V/R | 0.3 N m |
| Speed at 0.8 s | approximately 2,121.990 RPM |
| Current at 0.8 s | approximately 0.444645 A |

Before 0.05 s, all states and outputs are zero for the default no-load case.
After the step:

- **RPM:** rises smoothly and monotonically for these parameters. Speed
  cannot jump through a finite inertia. It enters roughly the final 2% speed
  band about 0.287 s after the step.
- **Current:** starts continuously at zero, rises rapidly to approximately
  5.743 A, then decays as back EMF develops. Initial di/dt is V/La = 6,000 A/s.
  The startup peak is below the locked-rotor value because the shaft accelerates.
- **Electromagnetic torque:** has the same shape as current, scaled by Kt.
  Its peak is about 0.287 N m. It settles at a nonzero torque to balance friction.
- **Back EMF:** follows speed and rises toward 11.111 V. It is below the
  12 V supply because the remaining 0.889 V is the resistive drop at equilibrium.
- **Load torque:** remains zero in the default run; shaft inertia and viscous
  friction still affect the response.

Other parameter choices can yield different damping or transient shapes;
monotonic speed is not a universal property of every DC motor model.

The steady balances provide direct numerical checks:

\[
12=2(0.444444)+0.05(222.222),\qquad
0.05(0.444444)=0.0001(222.222)+0.
\]

Increasing a constant load to 0.05 N m at the same 12 V predicts 1,768.388 RPM
and 1.370370 A: speed falls and current rises. This relationship was also
checked by independent numerical integration. It is not closed-loop recovery.

**Signed-load caveat:** a nonzero constant TL applied while Va is still zero
will drive the ideal shaft backwards. That is consistent with an externally
applied signed torque; it does not represent static friction or a passive
load that always opposes motion. Keep TL = 0 for the initial validation.
To test a constant positive load without that pre-step interval, edit
`voltage_before_V` to 12 as well, so voltage is applied from t = 0. Load
pulses applied after startup will be added in the disturbance phase.

## Validation gate before Phase 3

The actual Simulink trajectory is compared at its logged times with the exact
constant-input solution on each side of the voltage step:

\[
x(t)=x_{ss}+e^{A(t-t_0)}[x(t_0)-x_{ss}],\qquad x_{ss}=-A^{-1}Bu.
\]

This is calculated with base MATLAB `expm`; it does not call the Simulink plant
or a transfer-function block. Checks cover:

1. Finite logged values and correct configured input values.
2. Current and speed agreement with the exact state-space trajectory.
3. Torque, back-EMF and RPM conversion identities.
4. Final equilibrium and electrical/mechanical balances, provided at least
   eight slow time constants have elapsed after the voltage step.
5. Integrated energy conservation for Kt = Ke in consistent SI units:

   \[
   \int V_ai\,dt=\Delta(\tfrac12L_ai^2+\tfrac12J\omega^2)
   +\int(R_ai^2+b\omega^2+T_L\omega)\,dt.
   \]

6. Nonnegative current/speed and the V/R current bound in the default
   zero-state, positive-step, no-load test.

Any FAIL causes an error after results have been saved. SKIP is explicitly
reported and does not count as successful validation. A short run may be
physically correct but insufficient to verify steady state. Coarse solver
steps can also degrade the trapezoidal energy check.

### What has actually been tested

MATLAB and Simulink are unavailable in the preparation environment. The `.m`
files have been reviewed but **have not been executed or compiled in MATLAB**.
The `.slx` layout and logging behavior must be verified on your installation.

The reference equations were independently tested in Python using SciPy ODE
integration against a matrix-exponential solution, with values read from the
numeric assignments in `motioncore_init.m`. The maximum current and speed
differences were below 6e-11 in A and rad/s respectively. Integrated energy
relative residual was approximately 1.68e-6 on the reference output grid.
These results validate the mathematical/numerical reference, not the Simulink
builder. The PNG and JSON in this package are explicitly reference artifacts.

An exact reference can also be plotted in **MATLAB without Simulink**:

```matlab
motioncore_init
t = linspace(0, mc.sim.stop_time_s, 4001).';
x = motioncore_reference(mc,t);
plot(t, x(:,2)*mc.units.rad_s_to_rpm); grid on
xlabel('Time (s)'); ylabel('Speed (RPM)');
```

### Modeling limits

The driver is an ideal voltage source, with no current limit, voltage drop or
bus sag. Resistance has no thermal drift. Magnetic saturation, brush friction,
Coulomb friction, cogging, gearbox compliance and supply limits are omitted.
J includes reflected attached inertia; TL must not duplicate J*domega/dt.
The 5.74 A startup current is plausible for these equations but is not a claim
that any chosen physical motor or driver can safely tolerate it. Identify
actual motor parameters and ratings before moving to hardware.

## If MATLAB reports an error

Send the **complete error text and stack with line numbers**, the output of
`version`, and `ver('simulink')`. If the model builds, also send the printed
check results and plot. Keep the current scripts and saved model so a fix can
target the failing block or parameter without changing the architecture.

Useful checks from the Command Window:

```matlab
which build_motioncore -all
version
ver('simulink')
```

If a build fails partway, its partially constructed model remains available
for inspection. Close it without saving after capturing the error, then rerun
the corrected builder. A name-conflict error means another model of the same
name is already loaded. An unsaved-model error means manual changes need to be
saved or the model explicitly closed first.

## Official API references consulted

- [Programmatic model editing](https://www.mathworks.com/help/simulink/programmatic-modeling.html)
- [add_block](https://www.mathworks.com/help/simulink/slref/add_block.html)
- [add_line](https://www.mathworks.com/help/simulink/slref/add_line.html)
- [Step block](https://www.mathworks.com/help/simulink/slref/step.html)
- [Integrator block](https://www.mathworks.com/help/simulink/slref/integrator.html)
- [To Workspace logging](https://www.mathworks.com/help/simulink/slref/toworkspace.html)
- [Model-workspace assignin](https://www.mathworks.com/help/simulink/slref/simulink.modelworkspace.assignin.html)
- [Simulation output](https://www.mathworks.com/help/simulink/gui/singlesimulationoutput.html)

The builder uses the longstanding Step parameter names `Before` and `After`.
MathWorks documents their R2026a replacements as `InitialValue` and
`FinalValue`, and states that existing applications continue to work.

Stop at the open-loop validation gate. Add Phase 3 only once the actual
Simulink run and its engineering checks have been reviewed.
