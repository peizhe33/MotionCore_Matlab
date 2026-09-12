function d = motioncore_phase4_reference(mc,p3,p4,varargin)
% Toolbox-independent averaged-PWM reference.
% This is the Phase 3 sampled controller/plant recurrence plus
% D = sat(Vcmd/Vdc), Va_avg = D*Vdc. Quantisation is optional and is OFF for
% the primary Phase 4 model.
if nargin>=4 && ~isempty(varargin{1}),quantize=logical(varargin{1});
else, quantize=logical(p4.quantized_duty_experiment); end
base = motioncore_phase3_reference(mc,p3);
d = base;
d.voltage_command_sat_V = base.voltage_V;
d.duty_ideal = min(max(base.voltage_V/p4.Vdc_V,0),1);
d.duty_cycle = d.duty_ideal;
if quantize
    levels = 2^p4.PWM_BITS-1;
    d.duty_cycle = round(d.duty_ideal*levels)/levels;
end
d.voltage_avg_V = d.duty_cycle*p4.Vdc_V;
d.pwm_error_V = d.voltage_avg_V-d.voltage_command_sat_V;
% Keep legacy voltage_V as the Phase 3 saturated controller signal.
end
