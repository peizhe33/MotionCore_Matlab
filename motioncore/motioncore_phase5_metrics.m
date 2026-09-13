function m = motioncore_phase5_metrics(d,s,e,p3,p5,isNominal)
if nargin<6,isNominal=true;end
if isNominal
    m=motioncore_phase3_metrics(d,p3); target=p3.reference_rpm;
else
    target=p3.reference_rpm+p3.second_step_delta_rpm;
    m=motioncore_phase3_metrics(d,p3,p3.second_step_s,p3.reference_rpm,target);
end
% Uniform Ts samples for RMS/means; dense solver grid for physical peaks.
tail=s.time_s>=p3.stop_time_s-p5.tail_window_s-1e-10;
et=e.time_s>=p3.stop_time_s-p5.tail_window_s-1e-10;
err=s.encoder_rpm_error;
m.steady_error_rpm=target-mean(s.rpm(tail));
m.tail_max_error_rpm=max(abs(target-s.rpm(tail)));
m.true_rpm_tail_ripple=max(s.rpm(tail))-min(s.rpm(tail));
m.encoder_max_abs_error_rpm=max(abs(err));
m.encoder_rms_error_rpm=sqrt(mean(err.^2));
m.encoder_mean_error_rpm=mean(err);
m.encoder_tail_ripple_rpm=max(s.encoder_rpm(tail))-min(s.encoder_rpm(tail));
m.encoder_tail_rms_error_rpm=sqrt(mean(err(tail).^2));
m.encoder_tail_mean_error_rpm=mean(err(tail));
m.delta_count_min=min(e.encoder_delta_count);
m.delta_count_max=max(e.encoder_delta_count);
m.tail_counts_per_window=mean(e.encoder_delta_count(et));
m.expected_counts_per_window=target*p5.ENCODER_CPR*p5.Tenc/60;
m.tail_delta_count_min=min(e.encoder_delta_count(et));
m.tail_delta_count_max=max(e.encoder_delta_count(et));
m.rpm_per_count=p5.rpm_per_count; m.encoder_ppr=p5.ENCODER_PPR;
m.encoder_cpr=p5.ENCODER_CPR; m.Tenc_s=p5.Tenc;
m.min_duty=min(d.duty_cycle); m.max_duty=max(d.duty_cycle);
m.duty_tail_ripple=max(s.duty_cycle(tail))-min(s.duty_cycle(tail));
m.saturation_percentage=100*m.saturation_duration_s/(d.time_s(end)-d.time_s(1));
% Quantisation is bounded relative to WINDOW-AVERAGE speed, not instantaneous.
windowRPM=diff(e.shaft_angle_rad)*60/(2*pi*p5.Tenc);
m.window_quantization_max_error_rpm=max(abs(e.encoder_rpm(2:end)-windowRPM));
end
