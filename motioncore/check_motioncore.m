function checks = check_motioncore(mc, d)
% CHECK_MOTIONCORE Compare logged states to an independent analytic solution.
% SKIP is explicit: a short simulation cannot validate final equilibrium.
xRef = motioncore_reference(mc, d.time_s);
states = [d.current_A d.omega_rad_s];
scale = max(max(abs(xRef),[],1), [1e-3 1]);
err = max(abs(states-xRef),[],1)./scale;
checks = struct('name', {}, 'status', {}, 'detail', {});
loggedValues = d{:,:};
add('Finite logged signals', all(isfinite(loggedValues(:))), 'All seven signals');
add('Current vs exact state-space solution', err(1) < mc.check.state_relative_tol, ...
    sprintf('Normalized max error %.3g', err(1)));
add('Speed vs exact state-space solution', err(2) < mc.check.state_relative_tol, ...
    sprintf('Normalized max error %.3g', err(2)));
add('Torque = Kt*i', closeEnough(d.torque_Nm, mc.motor.Kt*d.current_A), 'SI scaling');
add('Back EMF = Ke*omega', closeEnough(d.back_emf_V, mc.motor.Ke*d.omega_rad_s), 'SI scaling');
add('RPM conversion', closeEnough(d.rpm, mc.units.rad_s_to_rpm*d.omega_rad_s), '60/(2*pi)');
expectedV = mc.input.voltage_before_V + ...
    (mc.input.voltage_after_V-mc.input.voltage_before_V)* ...
    (d.time_s >= mc.input.step_time_s);
% Exclude only the discontinuity itself; solvers may log either event side.
away = abs(d.time_s-mc.input.step_time_s) > 1e-10;
add('Voltage step and constant load', ...
    closeEnough(d.voltage_V(away), expectedV(away)) && ...
    closeEnough(d.load_Nm, mc.input.load_torque_Nm+zeros(size(d.load_Nm))), ...
    'Configured source values');
elapsed = d.time_s(end)-mc.input.step_time_s;
if elapsed >= 8*mc.math.slow_tau_s
    eqErr = abs(states(end,:)-mc.math.final_state.')./ ...
        max(abs(mc.math.final_state.'), [1e-3 1]);
    add('Final equilibrium', all(eqErr < mc.check.steady_relative_tol), ...
        sprintf('Relative i/omega errors %.3g / %.3g', eqErr));
    vResidual = d.voltage_V(end)-mc.motor.Ra*d.current_A(end)-d.back_emf_V(end);
    tResidual = d.torque_Nm(end)-mc.motor.b*d.omega_rad_s(end)-d.load_Nm(end);
    add('Steady electrical and mechanical balances', ...
        abs(vResidual) < mc.check.steady_relative_tol*max(abs(d.voltage_V(end)),1) && ...
        abs(tResidual) < mc.check.steady_relative_tol*max(abs(d.torque_Nm(end)),1e-3), ...
        sprintf('Residuals %.3g V, %.3g N*m', vResidual,tResidual));
else
    checks(end+1) = struct('name','Final equilibrium and balances', ...
        'status','SKIP','detail','Increase stop time: less than 8 slow time constants after step.');
end
% Passivity/energy check for an ideal reciprocal PM motor (Kt == Ke in SI).
if abs(mc.motor.Kt-mc.motor.Ke) <= 1e-12*max(mc.motor.Kt,mc.motor.Ke)
    stored = 0.5*mc.motor.La*d.current_A.^2 + 0.5*mc.motor.J*d.omega_rad_s.^2;
    pin = d.voltage_V.*d.current_A;
    lossAndLoad = mc.motor.Ra*d.current_A.^2 + ...
        mc.motor.b*d.omega_rad_s.^2 + d.load_Nm.*d.omega_rad_s;
    Ein = trapz(d.time_s,pin);
    balance = Ein-trapz(d.time_s,lossAndLoad)-(stored(end)-stored(1));
    normEnergy = max([trapz(d.time_s,abs(pin)),abs(stored(end)-stored(1)),1e-6]);
    add('Energy conservation', abs(balance)/normEnergy < mc.check.energy_relative_tol, ...
        sprintf('Integrated residual %.3g J (%.3g relative)',balance,abs(balance)/normEnergy));
else
    checks(end+1) = struct('name','Energy conservation','status','SKIP', ...
        'detail','Kt and Ke differ in SI; review motor reciprocity/units.');
end
% Check the initial no-load positive-step case for motoring signs and bounds.
if mc.initial.current_A == 0 && mc.initial.omega_rad_s == 0 && ...
        mc.input.voltage_before_V == 0 && mc.input.voltage_after_V > 0 && ...
        mc.input.load_torque_Nm == 0
    add('No-load motoring signs and current bound', ...
        min(d.current_A) >= -1e-7 && min(d.omega_rad_s) >= -1e-7 && ...
        max(d.current_A) <= 1.001*mc.input.voltage_after_V/mc.motor.Ra, ...
        'Nonnegative speed/current; peak current no greater than V/R');
end
for n = 1:numel(checks)
    fprintf('[%s] %s: %s\n',checks(n).status,checks(n).name,checks(n).detail);
end

    function add(name, pass, detail)
        status = 'FAIL';
        if pass, status = 'PASS'; end
        checks(end+1) = struct('name',name,'status',status,'detail',detail);
    end
end

function yes = closeEnough(a,b)
yes = all(abs(a-b) <= 1e-9*max(1,max(abs(b))));
end
