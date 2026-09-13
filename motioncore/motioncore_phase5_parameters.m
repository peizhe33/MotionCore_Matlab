function p5 = motioncore_phase5_parameters(p5,p3)
% Recompute dependent values after an optional experiment override.
assert(isscalar(p5.ENCODER_PPR) && isfinite(p5.ENCODER_PPR) && ...
    p5.ENCODER_PPR>=1 && p5.ENCODER_PPR==fix(p5.ENCODER_PPR));
assert(ismember(p5.decode_multiplier,[1 2 4]));
assert(isfinite(p5.Tenc) && p5.Tenc>=p3.Ts && ...
    abs(p5.Tenc/p3.Ts-round(p5.Tenc/p3.Ts))<1e-10, ...
    'MotionCore:EncoderRate','Phase 5A supports Tenc = M*Ts, integer M >= 1.');
assert(isfinite(p5.theta_initial_rad));
p5.ENCODER_CPR=p5.decode_multiplier*p5.ENCODER_PPR;
p5.rpm_per_count=60/(p5.ENCODER_CPR*p5.Tenc);
p5.check.reference_rpm_tol=2*p5.rpm_per_count;
end
