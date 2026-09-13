"""Independent numerical Phase 5A check; does NOT execute MATLAB/Simulink.

Loads parameters from the user's saved Phase 3 MAT result. Compares exact
augmented ZOH propagation with event-separated solve_ivp integration, including
the angle integrator. Uses the encoder count difference for controller feedback.
Run: python verify_motioncore_phase5.py --output results_phase5_reference
Requires NumPy, SciPy and matplotlib; not needed by the MATLAB runner.
"""
from pathlib import Path
import argparse
import csv
import json
import hashlib
import zipfile
import xml.etree.ElementTree as XML

import numpy as np
from scipy.io import loadmat
from scipy.linalg import expm
from scipy.integrate import solve_ivp


def inspect_phase4(root):
    """Read the actual SLX root graph. No archive contents are modified."""
    model=root/'MotionCore_Phase4.slx'
    with zipfile.ZipFile(model) as z:
        system=XML.fromstring(z.read('simulink/systems/system_root.xml'))
        blocks={b.get('SID'):b.get('Name') for b in system.findall('Block')}
        edges={}
        for line in system.findall('Line'):
            source=line.find("P[@Name='Src']").text
            sid,port=source.split('#out:')
            for destination in line.findall(".//P[@Name='Dst']"):
                dst,number=destination.text.split('#in:')
                key=f'{blocks[dst]}/{number}'
                assert key not in edges
                edges[key]=f'{blocks[sid]}/{port}'
        expected_sources=['DC_Motor_Plant/1','DC_Motor_Plant/2','DC_Motor_Plant/3',
            'DC_Motor_Plant/4','DC_Motor_Plant/5','Voltage_Command/1','Load_Torque/1',
            'Reference/1','Feedback/1','Digital_PID/2','Digital_PID/1','Digital_PID/3',
            'Digital_PID/4','Digital_PID/5','Voltage_Command/1','PWM_Driver/2',
            'PWM_Driver/1','PWM_Average_Error/1']
        for n,source in enumerate(expected_sources,1):
            assert edges[f'Logging/{n}']==source
        expected={'Digital_PID/1':'Reference/1','Digital_PID/2':'Feedback/1',
            'Digital_PID/3':'Voltage_Command/1','DC_Motor_Plant/1':'PWM_Driver/1',
            'Feedback/1':'DC_Motor_Plant/1','PWM_Driver/1':'Voltage_Command/1'}
        for dst,src in expected.items(): assert edges[dst]==src
        feedback=next(b for b in system.findall('Block') if b.get('Name')=='Feedback')
        feedback_system=XML.fromstring(z.read('simulink/systems/'+feedback.find('System').get('Ref')+'.xml'))
        assert {b.get('Name') for b in feedback_system.findall('Block')}=={'Shaft_RPM','Sample_Speed','Sampled_RPM'}
    saved=loadmat(root/'results_phase4/phase4_validation.mat',simplify_cells=True,
                  variable_names=['p3','p4','mc','allPassed'])
    assert bool(saved['allPassed'])
    with (root/'results_phase4/phase3_phase4_regression.csv').open() as f:
        regressions=list(csv.DictReader(f))
    assert len(regressions)==4 and all(float(row['all_passed'])==1 for row in regressions)
    hashes={f.name:hashlib.sha256(f.read_bytes()).hexdigest()
            for f in list(root.glob('*.slx'))+list(root.glob('*.m')) if 'phase5' not in f.name.lower()}
    return dict(status='SAVED_BASELINE_AND_INTERFACE_REVIEW_NOT_NEW_SIMULATION',
        phase4_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
        expected_legacy_logging_inputs=len(expected_sources), saved_phase4_all_passed=True,
        max_saved_phase3_vs_phase4_rpm=max(float(r['max_abs_rpm']) for r in regressions),
        frozen_file_hashes=hashes)


def simulate(mc, p, target, ppr=1024, ratio=1, encoder=True, ode=False,
             stress=False, theta0=0.0):
    ts = float(p['Ts'])
    stop = 1.6 if stress else float(p['stop_time_s'])
    A = np.zeros((3, 3)); A[:2, :2] = p['A']; A[2, 1] = 1
    B = np.vstack((p['B'], [0, 0])).astype(float)
    matrix = np.zeros((5, 5)); matrix[:3, :3] = A; matrix[:3, 3:] = B
    E = expm(matrix*ts)
    rpm_scale = float(mc['units']['rad_s_to_rpm'])
    resolution = 60 / (4*ppr*ratio*ts)
    x = np.array([mc['initial']['current_A'], mc['initial']['omega_rad_s'], theta0], dtype=float)
    integral = float(p['integrator_initial_V'])
    prev_count = np.floor(theta0*4*ppr/(2*np.pi)); count = prev_count
    delta = 0; yenc = 0.0
    rows = []
    for k in range(round(stop/ts)+1):
        t = k*ts
        if k % ratio == 0:
            count = np.floor(x[2]*4*ppr/(2*np.pi))
            delta = count-prev_count; prev_count = count; yenc = delta*resolution
        rpm = x[1]*rpm_scale
        ref = target if t >= float(p['step_time_s'])-1e-12 else float(p['initial_reference_rpm'])
        if stress and t >= float(p['second_step_s'])-1e-12:
            ref -= 2000
        error = ref-(yenc if encoder else rpm)
        raw = float(p['Kp'])*error+integral
        u = np.clip(raw, 0.0, float(p['voltage_max_V']))
        duty = np.clip(u/12, 0, 1); va = duty*12
        rows.append([t, ref, rpm, yenc, x[0], x[1], x[2], count, delta,
                     error, raw, u, duty, integral, va])
        integral += ts*(float(p['Ki'])*error+float(p['Kb'])*(u-raw))
        inp = np.array([va, p['load_Nm']], dtype=float)
        if ode:
            sol = solve_ivp(lambda _, state: A@state+B@inp, (0, ts), x,
                            rtol=2e-11, atol=2e-13, max_step=ts/10)
            assert sol.success
            x = sol.y[:, -1]
        else:
            x = E[:3, :3]@x+E[:3, 3:]@inp
    keys = ('time_s reference_rpm true_rpm encoder_rpm current_A omega_rad_s '
            'shaft_angle_rad encoder_count delta_count error_rpm raw_voltage_V '
            'voltage_V duty_cycle integrator_V voltage_avg_V').split()
    arr = np.asarray(rows)
    return dict(zip(keys, arr.T)), resolution


def metrics(d, q, p, ppr, ratio=1, stress=False):
    ts = float(p['Ts']); t = d['time_s']; rpm = d['true_rpm']; enc = d['encoder_rpm']
    tail = t >= t[-1]-.2-1e-12
    target = d['reference_rpm'][-1]
    event = p['second_step_s'] if stress else p['step_time_s']
    start = 3000 if stress else p['initial_reference_rpm']
    post = t >= event-1e-12
    progress = (rpm[post]-start)/(target-start)
    def crossing(level):
        ids = np.flatnonzero(progress >= level)
        if not len(ids): return np.nan
        j = ids[0]; tt = t[post]
        if j == 0: return tt[0]
        return tt[j-1]+(level-progress[j-1])*(tt[j]-tt[j-1])/(progress[j]-progress[j-1])
    outside = np.flatnonzero(np.abs(rpm[post]-target)>max(1, .02*abs(target-start)))
    idx = outside[-1]+1 if len(outside) else 0
    tt = t[post]
    settled = tt[idx]-event if idx < len(tt) and tt[-1]-tt[idx]>=.05 else np.nan
    hits = np.arange(0, len(t), ratio)
    window = np.diff(d['shaft_angle_rad'][hits])*60/(2*np.pi*ratio*ts)
    window_error = enc[hits][1:]-window
    sat = np.abs(d['raw_voltage_V']-d['voltage_V']) > 1e-8
    err = enc-rpm
    m = dict(target_rpm=float(target), ppr=ppr, cpr=4*ppr, Tenc_s=ratio*ts,
             rpm_per_count=q, rise_time_s=crossing(.9)-crossing(.1), settling_time_s=settled,
             overshoot_pct=max(0, 100*(np.max(progress)-1)),
             mean_steady_error_rpm=target-np.mean(rpm[tail]), max_steady_error_rpm=np.max(np.abs(target-rpm[tail])),
             peak_sampled_current_A=np.max(np.abs(d['current_A'])),
             min_duty=np.min(d['duty_cycle']), max_duty=np.max(d['duty_cycle']),
             saturation_duration_s=np.sum(sat[:-1])*ts,
             encoder_max_abs_error_rpm=np.max(np.abs(err)), encoder_rms_error_rpm=np.sqrt(np.mean(err**2)),
             encoder_mean_error_rpm=np.mean(err), encoder_tail_ripple_rpm=np.ptp(enc[tail]),
             true_tail_ripple_rpm=np.ptp(rpm[tail]),
             encoder_tail_rms_error_rpm=np.sqrt(np.mean(err[tail]**2)),
             mean_tail_counts_per_window=np.mean(d['delta_count'][hits][tail[hits]]),
             expected_counts_per_window=target*4*ppr*ratio*ts/60,
             max_window_quantization_error_rpm=np.max(np.abs(window_error)))
    assert all(np.all(np.isfinite(v)) for v in d.values())
    assert np.all(d['encoder_count']==np.floor(d['encoder_count']))
    assert np.all(np.diff(d['encoder_count'][hits])==d['delta_count'][hits][1:])
    assert np.all(d['encoder_rpm']==d['delta_count']*q)
    assert np.max(np.abs(window_error))<=q*(1+1e-8)
    assert np.all((d['duty_cycle']>=0) & (d['duty_cycle']<=1))
    assert np.max(np.abs(d['voltage_avg_V']-12*d['duty_cycle']))<1e-12
    assert np.max(np.abs(d['current_A']))<=6.12
    return {k: float(v) for k,v in m.items()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    saved = root/'results_phase3/phase3_validation.mat'
    data = loadmat(saved, simplify_cells=True, variable_names=['mc', 'p3'])
    mc, p = data['mc'], data['p3']
    output = args.output; output.mkdir(parents=True, exist_ok=True)
    baseline_review=inspect_phase4(root)
    (output/'baseline_interface_review.json').write_text(json.dumps(baseline_review,indent=2)+'\n')
    results = []; sweeps = []; exact_runs = {}; low = {}; comparisons = []
    for target in (500, 1000, 1500, 2000):
        d, q = simulate(mc,p,target)
        m = metrics(d,q,p,1024)
        assert abs(m['mean_steady_error_rpm'])<1 and m['max_steady_error_rpm']<3
        assert m['settling_time_s']<.5 and m['overshoot_pct']<5
        ideal, _ = simulate(mc,p,target,encoder=False)
        ode, _ = simulate(mc,p,target,ode=True)
        diff_rpm = float(np.max(np.abs(d['true_rpm']-ode['true_rpm'])))
        assert diff_rpm < .001
        comparisons.append(dict(target_rpm=target, exact_vs_ode_max_rpm=diff_rpm,
            phase4_vs_phase5_max_true_rpm=float(np.max(np.abs(d['true_rpm']-ideal['true_rpm']))),
            phase4_vs_phase5_rms_true_rpm=float(np.sqrt(np.mean((d['true_rpm']-ideal['true_rpm'])**2)))))
        results.append(m); exact_runs[target]=d
        with (output/f'reference_{target}rpm.csv').open('w',newline='') as f:
            writer=csv.writer(f); writer.writerow(d); writer.writerows(zip(*d.values()))
    for ppr in (128,256,1024,2048):
        for target in (50,100,250,500):
            d,q=simulate(mc,p,target,ppr=ppr); sweeps.append(metrics(d,q,p,ppr))
            if ppr==1024: low[target]=d
    stress,q=simulate(mc,p,3000,stress=True)
    stress_metrics=metrics(stress,q,p,1024,stress=True)
    assert stress_metrics['saturation_duration_s']>0 and stress_metrics['settling_time_s']<.5
    assert stress_metrics['max_steady_error_rpm']<3
    longer=[]
    for ratio in (2,5,10):
        d,q=simulate(mc,p,1000,ratio=ratio)
        longer.append(metrics(d,q,p,1024,ratio))
    # Boundary/startup behavior, including floor's signed-angle convention.
    d,q=simulate(mc,p,500,theta0=.731)
    assert d['delta_count'][0]==0
    angles=np.array([-2*np.pi, -.01, 0, .01, 2*np.pi])
    counts=np.floor(angles*4096/(2*np.pi))
    assert np.all(np.diff(counts)>0)
    report=dict(status='INDEPENDENT_NUMERICAL_REFERENCE_PASS_NOT_SIMULINK',
        baseline_review=baseline_review,
        parameter_source=str(saved.relative_to(root)),
        parameter_source_sha256=hashlib.sha256(saved.read_bytes()).hexdigest(),
        main=results, exact_vs_ode=comparisons, low_speed_ppr_sweep=sweeps,
        antiwindup=stress_metrics, longer_windows=longer)
    # Optional coarse encoders may not settle inside a 2% band; preserve that
    # result as JSON null rather than labelling the experiment successful.
    def json_safe(value):
        if isinstance(value, dict): return {k:json_safe(v) for k,v in value.items()}
        if isinstance(value, list): return [json_safe(v) for v in value]
        if isinstance(value, float) and not np.isfinite(value): return None
        return value
    (output/'independent_reference.json').write_text(json.dumps(json_safe(report),indent=2,allow_nan=False)+'\n')
    for name,rows in [('main_metrics',results),('low_speed_resolution',sweeps),('comparison',comparisons)]:
        with (output/f'{name}.csv').open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    for target,d in exact_runs.items():
        axes[0,0].plot(d['time_s'],d['true_rpm'],label=f'{target} RPM')
    axes[0,0].set(title='Independent reference: unchanged PI',xlabel='Time (s)',ylabel='True speed (RPM)')
    d=exact_runs[1000]; take=d['time_s']>=.95
    axes[0,1].step(d['time_s'][take],d['encoder_rpm'][take],where='post',label='Encoder')
    axes[0,1].plot(d['time_s'][take],d['true_rpm'][take],label='True')
    axes[0,1].set(title='1000 RPM: counting ripple (4096 CPR)',xlabel='Time (s)',ylabel='RPM')
    d=low[50]; take=d['time_s']>=.95
    axes[1,0].step(d['time_s'][take],d['encoder_rpm'][take],where='post',label='Encoder')
    axes[1,0].plot(d['time_s'][take],d['true_rpm'][take],label='True')
    axes[1,0].set(title='50 RPM: only 3.41 counts/window',xlabel='Time (s)',ylabel='RPM')
    for target in (50,100,250,500):
        rows=[r for r in sweeps if r['target_rpm']==target]
        axes[1,1].plot([r['ppr'] for r in rows],[r['encoder_tail_rms_error_rpm'] for r in rows],'-o',label=f'{target} RPM')
    axes[1,1].set(title='Resolution sweep, x4 decoding, 1 ms',xlabel='PPR',ylabel='Tail estimator RMS error (RPM)')
    for ax in axes.flat: ax.grid(alpha=.25); ax.legend(fontsize=8)
    fig.savefig(output/'independent_reference.png',dpi=150); plt.close(fig)
    print(json.dumps({'status':report['status'],'main':results,'comparison':comparisons},indent=2))


if __name__=='__main__': main()
