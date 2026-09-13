MotionCore Phase 5B — quadrature behavioral reference

Prepared against GitHub commit `ab7a68d` (validated Phase 5A). All 34 existing MATLAB scripts/models remain byte-identical. Only new Phase 5B files are supplied.

**Execution status:** the MATLAB source has passed static syntax analysis. An independent Python numerical implementation passed the decoder, four-speed, and saturation-recovery checks. MATLAB/Simulink is unavailable in the preparation environment: the new SLX has NOT been generated or simulated here, and Phase 5B is NOT yet declared complete. The included numerical results are explicitly labeled as independent reference results.

Run in MATLAB

Extract this package into your existing repository's `motioncore` directory, beside the validated scripts and four SLX files. Then run:

```matlab
cd('path/to/MotionCore_Matlab/motioncore')
summary = run_motioncore_phase5b;
```

MATLAB and Simulink are required. The new implementation uses standard Simulink blocks; it does not require Simscape, HDL Coder, Fixed-Point Designer, or Control System Toolbox. The old checkers use base-MATLAB `expm` for exact state transitions.

The runner reads the saved Phase 5A model workspace rather than guessing your current parameter values. It performs these gates in order:

1. Execute frozen open-loop, Phase 3, averaged Phase 4, and Phase 5A models with the original checkers, including established anti-windup tests. Stop on any failure; never rebuild or repair those phases.
2. Copy the WHOLE `MotionCore_Phase5.slx` file into `MotionCore_Phase5B.slx`. Check subsystem topology, all legacy logging connections, and all legacy sampled signals for the four reference speeds before adding any new blocks.
3. Execute 23 standalone tests using the actual Simulink decoder/generator block factories: all 16 transition pairs, forward/reverse/hold/invalid-resynchronization/direction-reversal sequences, and one full revolution in each direction.
4. Attach the new quadrature path, verify its connections and protected subsystem signatures, and run 500/1000/1500/2000 RPM plus the 3000→1000 RPM stress case.
5. Require all decoder, count, estimator, physical, PI, energy, saturation, tracking, and Phase 5A comparison checks to pass. Verify every frozen file remains byte-identical. Only then print `PHASE 5B PASS`.

Each run gets a unique `results_phase5b` subdirectory. It contains frozen-regression results, decoder tests, complete edge-rate MAT data, controller/estimator CSVs, A/B zoom CSVs, metrics, comparisons, four PNG/FIG plots, and the final validation MAT file. Only the 1000 RPM figure is opened by this runner. It does not close unrelated figures that you already had open. Existing Phase 5B SLX files are backed up before replacement; an unsaved loaded model causes an explicit stop.

Full edge logging means 1,000,001 edge samples per nominal one-second case and 1,600,001 in the stress case. Allow several GB of free RAM and disk for the complete run. Plots use a reduced selection of physical samples for display; validation and saved edge data are not decimated.

Architecture and preserved behavior

The existing `Encoder` subsystem remains intact. Its continuous shaft-angle output stimulates `Quadrature_AB`. Its former ideal count remains available for validation only. New subsystems are:

- `Quadrature_AB`: sample angle, calculate phase, emit Boolean A/B.
- `Quadrature_Decoder`: previous-state register, transition lookup, signed position register, invalid-transition counter.
- `Decoded_Count_Snapshot`: sample decoded position at `Tenc`, convert the snapshot to the unchanged estimator's double data type.

The active loop is shaft angle → Boolean A/B → x4 decoder → decoded count snapshot → existing `RPM_Estimator` → existing feedback sampler → existing PI → saturation → averaged PWM → golden motor plant. The decoder has exactly two inputs: A and B. No ideal count, true RPM, or angle enters the decoder or speed estimator.

The plant, PI, PWM, reference, feedback and estimator subsystems retain their original internals. All legacy logging blocks remain intact. The legacy `encoder_count` log now records the decoded snapshot actually used by the estimator; `ideal_encoder_count_phase5a` records the original ideal count. The full-rate `decoded_encoder_count` log records every updated decoder count.

Timing and parameters

| Quantity | Value / interpretation |
|---|---|
| PI sample period `Ts` | 1 ms, 1 kHz |
| `Kp`, `Ki`, `Kd` | Saved coefficients, approximately 0.00620355671, 0.0848230016, 0 |
| Anti-windup `Kb` | 20 s⁻¹; existing Forward-Euler integrator recurrence |
| Supply / duty | 12 V, 0…1, `Va = duty * Vdc` |
| PWM frequency | 20 kHz; actuator remains averaged, without simulated switching pulses |
| Encoder PPR | 1024 complete A cycles per mechanical revolution; likewise B |
| x4 CPR | 4096 valid A/B transitions per mechanical revolution |
| Edge observation / decoder period `Tedge` | 1 µs, 1 MHz |
| Estimator period `Tenc` | 1 ms |
| Estimator resolution | `60/(4096*0.001) = 14.6484375 RPM/count` |
| Behavioral position register | signed `int64`; conversion to double only after snapshot |
| Previous A/B state / transition delta | `uint8` / `int8` |
| Invalid-transition counter | `uint64` |

Transition frequency is `abs(RPM)*CPR/60`: 136,533.33 transitions/s at 2000 RPM. The analytical no-load speed is about 2122.07 RPM, giving 144,866.37 transitions/s. At the 3000 RPM design envelope it is 204,800 transitions/s, or 0.2048 counts per 1 µs observation. This provides over four observations per transition at the design envelope; the modeled 12 V motor cannot reach 3000 RPM without external driving torque.

The new model limits the continuous solver's maximum step to 1 µs while preserving its type and tolerances. A runtime guard checks actual angular advance, integer count changes, and speed envelope. Zero invalid transitions alone is insufficient: three skipped edges can resemble a valid reverse edge and four skipped edges can look like no movement.

At each edge tick the reference evaluates current A/B against the previous state, exposes the updated count, and stores it for the next tick. On a coincident 1 ms tick, the estimator snapshots that updated count. No extra pipeline delay is deliberately introduced. The block tests and full-stream checker verify this ordering; an unexpected sample delay must be diagnosed, not hidden by retuning or shifting logs.

Quadrature convention

Let `q = floor(theta * CPR/(2*pi))` and `phase = mod(q,4)`. Generator tables are `A=[0 0 1 1]`, `B=[0 1 1 0]`. Positive motion follows `00 → 01 → 11 → 10 → 00`: B leads A under this convention. Each full A/B electrical cycle contributes four counts. At constant speed both channels have 50% duty cycle and one-quarter-cycle separation. During acceleration their *time-domain* duty need not be exactly 50%; their angular duty is 50%.

State encoding is `state=2*A+B`. Lookup index is `4*previous_state + current_state` (zero based):

| Previous \ current | 00 | 01 | 10 | 11 |
|---|---:|---:|---:|---:|
| 00 | 0 | +1 | −1 | invalid |
| 01 | −1 | 0 | invalid | +1 |
| 10 | +1 | invalid | 0 | −1 |
| 11 | invalid | −1 | +1 | 0 |

Invalid transitions contribute zero position change, increment the diagnostic counter, and resynchronize the previous-state register to the observed state. `encoder_direction` is the signed **transition increment** (+1/−1/0), not a latched indication of the last direction.

Initial count and A/B phase are consistently preloaded from the configured initial angle. Default initial angle is zero. This is a simulation alignment convention: a physical incremental encoder does not provide absolute position at startup. Both forward and reverse one-revolution tests require exactly ±4096 counts.

The estimator remains `DeltaCount[k]=Count[k]-Count[k-1]`, `RPM[k]=DeltaCount[k]*60/(CPR*Tenc)`. Its first delta is zero because the previous-count register is preloaded consistently.

Comparison tolerances

Two comparisons have different meanings:

- **Same physical trajectory:** decoded count and the generator's ideal edge count must match exactly at every 1 µs tick, without a persistent offset. Against the preserved Phase 5A calculation sampled at 1 ms, only a one-count floating-point floor choice within 1e-7 counts of an exact integer boundary is allowed. Maximum, final, and RMS differences are reported.
- **Separate closed-loop simulations:** the frozen Phase 5A solver and Phase 5B solver can place a near-boundary angle on opposite sides of a count threshold. Allow at most one count difference; consecutive endpoint differences can cause at most two count units of estimator difference (29.296875 RPM). Limits are 1 RPM for true speed, 0.1 A for sampled current, 0.2 V for voltage commands, 0.2/12 for duty, and 0.02 V for integral state. These are conservative acceptance caps for quantization-boundary effects, not expected errors or permission to ignore a wiring fault. The true-speed cap is 0.2% of the lowest target; two estimator units correspond to about 0.182 V of proportional command, supporting the 0.2 V voltage cap.

The unchanged full-model copy has a separate strict numerical-identity gate: absolute tolerance 1e-7 plus relative tolerance 2e-6 per signal. All logged signals must exist and be finite; A/B logs must actually have logical data type and the position register must log as `int64`.

Independent numerical results prepared here

These are NOT Simulink validation results. The Python reference evaluates the exact continuous plant at every 1 µs observation, constructs Boolean A/B, and runs the transition decoder before sampling count feedback. It compares against the frozen Phase 5A Python count-feedback reference. A separate event-separated ODE integration checks the 1000 RPM physical trajectory.

| Target RPM | Rise 10–90%, s | Settle 2%, s | Overshoot, % | Tail max true error, RPM | Peak current, A |
|---|---:|---:|---:|---:|---:|
| 500 | 0.141835 | 0.252 | 0.01644 | 0.08219 | 1.51468 |
| 1000 | 0.141798 | 0.253 | 0.00550 | 0.05893 | 3.01013 |
| 1500 | 0.141811 | 0.253 | 0.00322 | 0.04827 | 4.53412 |
| 2000 | 0.144009 | 0.256 | 0.00264 | 0.05270 | 5.74290 |

All 23 independent decoder tests passed. All normal/stress cases had zero invalid transitions and zero decoded-versus-ideal count differences. Separate Phase 5A/5B count and encoder-RPM differences were also zero; maximum true-RPM difference was 6.83e-13 RPM. The independent ODE difference at 1000 RPM was 9.26e-11 RPM. Encoder tail ripple remained 14.6484375 RPM peak-to-peak, as expected from the unchanged one-count resolution.

The 3000→1000 RPM stress test saturated and recovered within about 0.149 s. Maximum integral magnitude was 10.274 V; tail maximum true-speed error was 0.094 RPM. These observations support the implementation design; only the MATLAB runner can validate the actual generated model and its scheduling.

To reproduce the supplementary reference:

```text
python verify_motioncore_phase5b.py
```

It requires NumPy, SciPy, matplotlib, the unchanged `verify_motioncore_phase5.py`, and saved Phase 5A results from the repository. It writes only `phase5b_reference_results`. This Python tool is not needed to run the MATLAB model.

Completion and RTL handoff

After the MATLAB runner passes, Phases 1–5B together validate the motor equations and energy balance, sampled PI and anti-windup, averaged PWM voltage mapping, count-window speed estimation, and x4 decoding within the tested speed/edge envelope. This completes the requested behavioral feature scope. Do not begin another MATLAB feature phase.

Recommended later RTL development and verification order:

1. `quadrature_decoder` and `position_counter`: implement the transition convention, reset alignment and invalid-transition policy; replay the 16 transition pairs, holds, reversals and one-revolution vectors.
2. `count_difference_register` and `rpm_estimator`: define simultaneous decoder/snapshot ordering, periodic enable generation, signed subtraction, scale rounding and overflow handling; compare against the saved count snapshots and RPM.
3. PI and PWM blocks: select arithmetic widths and coefficient formats, preserve anti-windup and saturation update ordering, verify each block against this behavioral reference.
4. Integrate the chain, account explicitly for synchronizer/pipeline latency, then verify closed-loop traces before board-level motor tests.

The generator is a simulation stimulus, not an RTL shaft-angle sensor. Future hardware needs two previous-state bits, a chosen signed position width and rollover policy, a signed window-difference width, an invalid-event-counter width, and an RPM scaling representation. At the 3000 RPM envelope, a 1 ms window contains about 205 counts; signed 9 bits holds that nominal delta, but practical margin, faults and modular subtraction require a deliberate wider choice. A signed 32-bit position counter would reach its positive limit in roughly 2.9 hours at 3000 RPM (4.4 hours at 2000 RPM), so wrap-safe difference arithmetic or a wider counter must be specified. The behavioral `int64` choice is not a finalized FPGA area decision.

Remaining limits: ideal A/B levels without input synchronization/noise, deterministic edge observation, no simulated hardware pipeline delay, double PI/estimator arithmetic, averaged power stage without switching ripple or dead time, and no motor-current limiter. Position-to-double conversion is checked to stay below 2^53 in magnitude. These are explicit boundaries of this reference; no RTL, fixed-point conversion, detailed switching, or additional tuning is included.
