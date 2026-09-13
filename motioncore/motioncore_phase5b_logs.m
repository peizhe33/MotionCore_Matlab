function [d,s,e,f] = motioncore_phase5b_logs(out,p3,p5,b)
%MOTIONCORE_PHASE5B_LOGS
% Preserve the validated Phase 5A logger while adding the complete
% Phase 5B quadrature edge-domain audit stream.
%
% Time domains:
%   b.Tedge  : quadrature generator / decoder processing interval
%   p5.Tenc  : encoder RPM-estimation interval
%   p3.Ts    : PI controller sample interval
%
% IMPORTANT:
% Digital encoder state/count signals are sampled using exact integer
% edge-tick mapping rather than interp1(). This avoids floating-point
% endpoint artefacts and more closely represents the eventual FPGA
% register-snapshot behaviour.

%% ------------------------------------------------------------------------
% Preserve validated Phase 5A logging
% -------------------------------------------------------------------------
[d,s,e] = motioncore_phase5_logs(out,p3,p5,'encoder');

%% ------------------------------------------------------------------------
% Phase 5B signals
% -------------------------------------------------------------------------

% Signals produced at EVERY quadrature-processing tick.
fast = { ...
    'encoder_A', ...
    'encoder_B', ...
    'quadrature_state', ...
    'previous_quadrature_state', ...
    'decoded_encoder_count', ...
    'encoder_direction', ...
    'invalid_transition_count', ...
    'ideal_count_edge', ...
    'encoder_edge_angle_rad'};

% Signals produced on the slower estimator grid.
slow = { ...
    'ideal_encoder_count_phase5a', ...
    'count_difference_vs_phase5a'};

%% ------------------------------------------------------------------------
% Construct complete edge-domain time grid
% -------------------------------------------------------------------------
t = (0:round(p3.stop_time_s / b.Tedge)).' * b.Tedge;

f = table(t,'VariableNames',{'time_s'});

%% ------------------------------------------------------------------------
% Read and validate EVERY edge-processing signal
%
% f = complete 1-us (or configured Tedge) audit stream
% s = controller-grid snapshot
% e = encoder-estimator-grid snapshot
%
% Do NOT interpolate digital encoder states/counts. Map target samples onto
% exact integer encoder ticks instead.
% -------------------------------------------------------------------------
for k = 1:numel(fast)

    n = fast{k};

    sig = required(out,n);

    [st,idx] = unique( ...
        snap(double(sig.Time(:)),b.Tedge), ...
        'last');

    v = double(sig.Data(:));
    v = v(idx);

    % Phase 5B requires the complete decoder history so skipped transitions
    % cannot disappear through decimation.
    assert( ...
        numel(st) == numel(t) && ...
        max(abs(st-t)) < b.Tedge*1e-5, ...
        'MotionCore:MissingEdgeSamples', ...
        'Log %s must contain EVERY edge-processing hit.',n);

    %% Preserve original signal-type checks

    if any(strcmp(n,{'encoder_A','encoder_B'}))
        assert( ...
            islogical(sig.Data), ...
            'MotionCore:EncoderType', ...
            'A/B must be Boolean.');
    end

    if strcmp(n,'decoded_encoder_count')
        assert( ...
            isa(sig.Data,'int64'), ...
            'MotionCore:EncoderType', ...
            'Position must be int64.');
    end

    if strcmp(n,'invalid_transition_count')
        assert( ...
            isa(sig.Data,'uint64'), ...
            'MotionCore:EncoderType', ...
            'Invalid counter must be uint64.');
    end

    %% Complete edge-domain audit stream
    f.(n) = v;

    %% Exact digital snapshots
    %
    % The estimator/controller observes the decoder register at an exact
    % processing tick. There is no continuous interpolation in the intended
    % FPGA implementation.
    s.(n) = sampleByTick( ...
        st,v,s.time_s,b.Tedge,n);

    e.(n) = sampleByTick( ...
        st,v,e.time_s,b.Tedge,n);
end

%% ------------------------------------------------------------------------
% Slower Phase 5A comparison signals
% -------------------------------------------------------------------------
for k = 1:numel(slow)

    n = slow{k};

    sig = required(out,n);

    [st,idx] = unique( ...
        snap(double(sig.Time(:)),p5.Tenc), ...
        'last');

    v = double(sig.Data(:));
    v = v(idx);

    assert( ...
        st(1) == 0 && ...
        st(end) >= p3.stop_time_s-1e-9, ...
        'MotionCore:IncompleteLog', ...
        'Incomplete log: %s',n);

    % Dense plant/logging grid:
    % hold the latest discrete estimator value.
    %
    % 'extrap' is intentional here because the discrete signal should hold
    % its final register value if the dense-time endpoint differs only by
    % floating-point scheduling tolerance.
    d.(n) = interp1( ...
        st,v,d.time_s,'previous','extrap');

    % s/e are discrete grids. Use exact tick mapping rather than
    % interpolation for deterministic digital behaviour.
    s.(n) = sampleByTick( ...
        st,v,s.time_s,p5.Tenc,n);

    e.(n) = sampleByTick( ...
        st,v,e.time_s,p5.Tenc,n);
end

end


%% =========================================================================
% Required-log reader
% =========================================================================
function sig = required(out,n)

assert( ...
    any(strcmp(who(out),n)), ...
    'MotionCore:MissingLog', ...
    'Missing required Phase 5B log: %s',n);

sig = out.get(n);

assert( ...
    isa(sig,'timeseries') && ...
    numel(sig.Time) > 1 && ...
    numel(sig.Data) == numel(sig.Time), ...
    'MotionCore:InvalidLog', ...
    'Invalid or empty scalar timeseries: %s',n);

assert( ...
    all(isfinite(double(sig.Data(:)))) && ...
    all(isfinite(double(sig.Time(:)))), ...
    'MotionCore:NonfiniteLog', ...
    'Nonfinite log: %s',n);

end


%% =========================================================================
% Snap numerical timestamps onto their intended discrete grid
% =========================================================================
function t = snap(t,Ts)

nearest = round(t/Ts)*Ts;

mask = abs(t-nearest) < Ts*1e-5;

t(mask) = nearest(mask);

end


%% =========================================================================
% Exact discrete-register snapshot using integer processing ticks
% =========================================================================
function y = sampleByTick(sourceTime,sourceData,targetTime,Ts,name)

sourceTime = sourceTime(:);
sourceData = sourceData(:);
targetTime = targetTime(:);

% Convert floating-point timestamps into exact integer clock ticks.
sourceTick = int64(round(sourceTime/Ts));
targetTick = int64(round(targetTime/Ts));

% Ensure the requested target grid itself really lies on the intended
% digital clock domain.
targetGridTime = double(targetTick)*Ts;

assert( ...
    all(abs(targetTime-targetGridTime) < Ts*1e-5), ...
    'MotionCore:OffGridSnapshot', ...
    'Requested samples for %s are not aligned to the %.12g s digital grid.', ...
    name,Ts);

% Every estimator/controller sample must correspond to an actual retained
% decoder-processing tick.
[found,loc] = ismember(targetTick,sourceTick);

assert( ...
    all(found), ...
    'MotionCore:MissingSnapshot', ...
    'Unable to map every requested sample of %s onto the %.12g s digital grid.', ...
    name,Ts);

y = sourceData(loc);

assert( ...
    numel(y) == numel(targetTime), ...
    'MotionCore:SnapshotLength', ...
    'Unexpected snapshot length for %s.',name);

assert( ...
    all(isfinite(y)), ...
    'MotionCore:NonfiniteSnapshot', ...
    'Nonfinite sampled value found for %s.',name);

end