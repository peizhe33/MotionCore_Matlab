"""Independent numerical reference ONLY; does not execute Simulink.
Run beside the frozen scripts: python verify_motioncore_phase5b.py
Requires numpy, scipy, matplotlib. Writes only phase5b_reference_results.
"""
from pathlib import Path
import csv
import hashlib
import json
import shutil
import zipfile
import xml.etree.ElementTree as ET
import numpy as np
from scipy.io import loadmat
from scipy.linalg import expm
from verify_motioncore_phase5 import simulate as phase5a, metrics

ROOT = Path(__file__).resolve().parent
EDGE = 1e-6
CPR = 4096
DELTA = np.array([0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0])
INVALID = np.array([0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0])
GRAY = np.array([0, 1, 3, 2])


def decode(states, previous=0, count=0):
    prev = np.r_[previous, states[:-1]]
    index = 4*prev+states
    return count+np.cumsum(DELTA[index]), DELTA[index], INVALID[index]


def units():
    tests = []
    for prev in range(4):
        for cur in range(4):
            count, step, bad = decode(np.array([prev, cur]), prev)
            movement = (int(GRAY[cur])-int(GRAY[prev])) % 4  # inverse Gray permutation
            want = 1 if movement == 1 else -1 if movement == 3 else 0
            assert count[-1] == want and step[-1] == want and bad[-1] == (movement == 2)
            tests.append(f'transition_{prev}_{cur}')
    for name, states, want, bads in [
        ('forward', [0, 1, 3, 2, 0], 4, 0), ('reverse', [0, 2, 3, 1, 0], -4, 0),
        ('hold', [0, 0, 0, 0], 0, 0), ('invalid_resync', [0, 3, 2, 0], 2, 1),
        ('reversal', [0, 1, 3, 2, 0, 2, 3, 1, 0], 0, 0)]:
        count, _, bad = decode(np.array(states))
        assert count[-1] == want and bad.sum() == bads
        tests.append(name)
    for direction in (1, -1):
        ideal = np.floor(.3+direction*np.arange(CPR*8+1)/8).astype(np.int64)
        states = GRAY[ideal % 4]
        count, _, bad = decode(states)
        assert np.array_equal(count, ideal) and count[-1] == direction*CPR and not bad.any()
        a, b = states//2, states % 2
        assert a[:-1].mean() == .5 and b[:-1].mean() == .5
        assert np.array_equal(b[:-1], np.roll(a[:-1], -direction*8))
        tests.append(f'one_revolution_{direction}')
    # Skipping 3 edges can look like a valid reverse step; 4 can be invisible.
    for skipped in (2, 3, 4):
        _, _, bad = decode(GRAY[np.array([0, skipped]) % 4])
        assert bool(bad[-1]) == (skipped == 2)
        assert skipped > 1  # separate physical edge-advance guard rejects ALL
    return tests


def simulate(mc, p, target, stress=False):
    ts = float(p['Ts']); n = round(ts/EDGE); assert n*EDGE == ts
    stop = 1.6 if stress else float(p['stop_time_s'])
    a = np.zeros((3, 3)); a[:2, :2] = p['A']; a[2, 1] = 1
    b = np.vstack((p['B'], [0, 0]))
    augmented = np.zeros((5, 5)); augmented[:3, :3] = a; augmented[:3, 3:] = b
    # Exact physical state at EVERY edge observation, while voltage is held.
    propagators = np.array([expm(augmented*j*EDGE)[:3] for j in range(1, n+1)])
    x = np.array([mc['initial']['current_A'], mc['initial']['omega_rad_s'], 0.0])
    integral = float(p['integrator_initial_V']); count = previous_count = state = 0
    invalid = 0; peak_current = 0.0; max_advance = 0.0; max_count_difference = 0
    rows = []; zoom = []; scale = float(mc['units']['rad_s_to_rpm']); q = 60/(CPR*ts)
    for k in range(round(stop/ts)+1):
        t = k*ts
        delta = count-previous_count; previous_count = count; enc = delta*q
        ref = target if t >= float(p['step_time_s'])-1e-12 else float(p['initial_reference_rpm'])
        if stress and t >= float(p['second_step_s'])-1e-12: ref -= 2000
        error = ref-enc; raw = float(p['Kp'])*error+integral
        u = np.clip(raw, 0, float(p['voltage_max_V'])); duty = u/12; va = duty*12
        rows.append([t, ref, x[1]*scale, enc, x[0], x[1], x[2], count, delta,
                     error, raw, u, duty, integral, va])
        if k == round(stop/ts): break
        integral += ts*(float(p['Ki'])*error+float(p['Kb'])*(u-raw))
        physical = propagators @ np.r_[x, va, p['load_Nm']]
        phi = physical[:, 2]*(CPR/(2*np.pi))
        ideal = np.floor(phi).astype(np.int64)
        states = GRAY[ideal % 4]  # A/B generator
        A, B = (states//2).astype(bool), (states % 2).astype(bool)
        # Decoder receives only A/B, never angle or the ideal count.
        decoded, direction, bad = decode(2*A.astype(int)+B.astype(int), state, count)
        assert np.array_equal(decoded, ideal), 'Decoded-vs-ideal offset'
        advance = np.diff(np.r_[x[2]*(CPR/(2*np.pi)), phi])
        assert np.max(np.abs(advance)) < 1
        assert np.max(np.abs(np.diff(np.r_[int(np.floor(x[2]*(CPR/(2*np.pi)))), ideal]))) <= 1
        invalid += int(bad.sum()); assert invalid == 0
        max_count_difference = max(max_count_difference, int(np.max(np.abs(decoded-ideal))))
        max_advance = max(max_advance, float(np.max(np.abs(advance))))
        peak_current = max(peak_current, float(np.max(np.abs(physical[:, 0]))))
        edge_t = t+np.arange(1, n+1)*EDGE
        take = (edge_t >= .8-1e-12) & (edge_t <= .8001+1e-12)
        if take.any(): zoom.extend(np.c_[edge_t[take], A[take], B[take], decoded[take]].tolist())
        x = physical[-1]; count = int(decoded[-1]); state = int(states[-1])
    keys = ('time_s reference_rpm true_rpm encoder_rpm current_A omega_rad_s '
            'shaft_angle_rad encoder_count delta_count error_rpm raw_voltage_V '
            'voltage_V duty_cycle integrator_V voltage_avg_V').split()
    d = dict(zip(keys, np.asarray(rows).T))
    extra = dict(peak_edge_grid_current_A=peak_current, invalid_transition_count=invalid,
                 max_same_trajectory_count_difference=max_count_difference,
                 max_counts_per_edge_tick=max_advance, max_integrator_V=float(np.max(np.abs(d['integrator_V']))))
    return d, q, extra, np.asarray(zoom)


def inspect_baseline():
    candidates = sorted(ROOT.glob('results_phase5/*/phase5_validation.mat'))
    assert candidates, 'Run alongside the validated repository including Phase 5A results.'
    saved = candidates[-1]
    data = loadmat(saved, simplify_cells=True, variable_names=['mc', 'p3', 'p4', 'p5', 'allPassed'])
    assert data['allPassed'] == 1
    assert data['p5']['ENCODER_CPR'] == CPR and data['p5']['Tenc'] == .001
    with zipfile.ZipFile(ROOT/'MotionCore_Phase5.slx') as z:
        sys = ET.fromstring(z.read('simulink/systems/system_root.xml'))
        names = {b.get('SID'): b.get('Name') for b in sys.findall('Block')}
        edges = {}
        for line in sys.findall('Line'):
            sid, port = line.find("P[@Name='Src']").text.split('#out:')
            for dst in line.findall(".//P[@Name='Dst']"):
                dsid, dp = dst.text.split('#in:'); edges[f'{names[dsid]}/{dp}'] = f'{names[sid]}/{port}'
        assert edges['RPM_Estimator/1'] == 'Encoder/1'
        assert edges['Feedback/1'] == 'RPM_Estimator/1' and edges['Digital_PID/2'] == 'Feedback/1'
        assert sum(k.startswith('Logging/') for k in edges) == 23
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in list(ROOT.glob('*.m'))+list(ROOT.glob('*.slx')) if 'phase5b' not in p.name.lower()}
    return data, dict(saved_result=str(saved.relative_to(ROOT)), saved_allPassed=True,
                      saved_result_sha256=hashlib.sha256(saved.read_bytes()).hexdigest(),
                      frozen_file_hashes=hashes, baseline_commit='ab7a68d')


def write_csv(path, rows):
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)


def main():
    out = ROOT/'phase5b_reference_results'; out.mkdir(exist_ok=True)
    data, baseline = inspect_baseline(); mc, p = data['mc'], data['p3']
    tests = units(); rows = []; comparisons = []; runs = {}
    for target, stress in [(500, False), (1000, False), (1500, False), (2000, False), (3000, True)]:
        d, q, extra, zoom = simulate(mc, p, target, stress)
        a, _ = phase5a(mc, p, target, stress=stress)
        m = metrics(d, q, p, 1024, stress=stress); m.update(extra); m['stress'] = stress
        assert abs(m['mean_steady_error_rpm']) < 1 and m['max_steady_error_rpm'] < 3
        assert m['settling_time_s'] < .5 and m['peak_edge_grid_current_A'] < 6.12
        if not stress: assert m['overshoot_pct'] < 5
        else: assert m['saturation_duration_s'] > 0 and m['max_integrator_V'] < 20
        comparison = dict(target_rpm=target, stress=stress)
        limits = dict(true_rpm=1, encoder_rpm=2*q, current_A=.1, voltage_V=.2,
                      raw_voltage_V=.2, duty_cycle=.2/12, encoder_count=1, integrator_V=.02)
        for name, limit in limits.items():
            diff = d[name]-a[name]; err = float(np.max(np.abs(diff)))
            assert err <= limit, (target, name, err)
            comparison['max_abs_'+name] = err
            comparison['rms_'+name] = float(np.sqrt(np.mean(diff**2)))
            comparison['final_'+name] = float(diff[-1])
        rows.append(m); comparisons.append(comparison); runs[target] = (d, a, zoom)
        print(f'{target} RPM stress={stress}: peak current {extra["peak_edge_grid_current_A"]:.6f} A, count difference 0, invalid 0', flush=True)
    # Independently integrate the 1000 RPM physical model with solve_ivp,
    # separated at controller events (the existing reference's ODE option).
    ode, _ = phase5a(mc, p, 1000, ode=True)
    ode_error = float(np.max(np.abs(ode['true_rpm']-runs[1000][0]['true_rpm'])))
    assert ode_error < .001
    for name, digest in baseline['frozen_file_hashes'].items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == digest
    report = dict(status='INDEPENDENT_NUMERICAL_REFERENCE_PASS_NOT_SIMULINK',
                  matlab_available=bool(shutil.which('matlab')), baseline=baseline, decoder_tests=tests,
                  Tedge_s=EDGE, Tenc_s=.001, Ts_s=.001, main_and_stress=rows,
                  phase5a_vs_phase5b=comparisons, max_rpm_difference_vs_solve_ivp=ode_error)
    (out/'independent_reference.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    write_csv(out/'metrics.csv', rows); write_csv(out/'phase5a_vs_phase5b.csv', comparisons)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    d, a, zoom = runs[1000]; t = d['time_s']
    fig, ax = plt.subplots(3, 2, figsize=(12, 10), layout='constrained')
    fig.suptitle('Phase 5B independent numerical reference — 1000 RPM (not a Simulink run)')
    ax[0, 0].plot(t, d['reference_rpm'], 'k--', label='Reference'); ax[0, 0].plot(t, d['true_rpm'], label='True')
    ax[0, 0].step(t, a['encoder_rpm'], where='post', color='green', ls=':', label='5A encoder')
    ax[0, 0].step(t, d['encoder_rpm'], where='post', alpha=.5, label='5B encoder'); ax[0, 0].set(ylabel='RPM')
    ax[0, 1].step((zoom[:, 0]-.8)*1e6, zoom[:, 1]+2, where='post', label='A (+2 offset)')
    ax[0, 1].step((zoom[:, 0]-.8)*1e6, zoom[:, 2], where='post', label='B')
    ax[0, 1].set(xlabel='Microseconds after 0.8 s', ylabel='Logic level / offset')
    ax[1, 0].step(t, d['encoder_count'], where='post', label='Decoded')
    ax[1, 0].step(t, np.floor(d['shaft_angle_rad']*(CPR/(2*np.pi))), where='post', ls='--', label='Ideal')
    ax[1, 0].set(ylabel='Count')
    ax[1, 1].plot(t, d['encoder_count']-np.floor(d['shaft_angle_rad']*(CPR/(2*np.pi))), label='Decoded minus ideal')
    ax[1, 1].set(ylabel='Count difference')
    ax[2, 0].step(t, d['encoder_rpm']-d['true_rpm'], where='post', label='Encoder minus true'); ax[2, 0].set(ylabel='RPM error')
    ax[2, 1].step(t, d['voltage_V'], where='post', label='Voltage (V)')
    ax[2, 1].plot(t, d['current_A'], label='Current (A)'); ax[2, 1].set(ylabel='Voltage / current')
    for axis in ax.flat: axis.grid(alpha=.25); axis.legend(fontsize=8)
    for axis in ax[1:, :].flat: axis.set_xlabel('Time (s)')
    fig.savefig(out/'reference_1000rpm.png', dpi=150); plt.close(fig)
    print(report['status']); print('Independent ODE RPM difference:', ode_error)


if __name__ == '__main__': main()
