function [checks,m] = check_motioncore_phase5b(mc,p3,p4,p5,b,d,s,e,f,isNominal)
if nargin<10,isNominal=true;end
% Legacy encoder_count now means the decoded snapshot used by the unchanged
% estimator. The original checker therefore still verifies the actual loop.
[checks,m]=check_motioncore_phase5(mc,p3,p4,p5,d,s,e,isNominal);
v=f{:,:}; add('Finite complete edge logs',all(isfinite(v(:))) && ...
    height(f)==round(p3.stop_time_s/b.Tedge)+1,'Every edge-processing tick retained');
A=f.encoder_A; B=f.encoder_B;
add('Binary A/B and state encoding',all(ismember(A,[0 1])) && all(ismember(B,[0 1])) && ...
    isequal(f.quadrature_state,2*A+B),'state = 2*A+B');
[count,step,invalid,previous]=motioncore_phase5b_decode_reference(f.quadrature_state,b.initial_state,b.initial_count);
add('x4 transition decoding and direction',isequal(f.encoder_direction,step) && ...
    isequal(f.previous_quadrature_state,previous) && isequal(f.decoded_encoder_count,count), ...
    'Independent Gray-cycle arithmetic versus implemented LUT/registers');
add('Invalid transitions diagnosed',isequal(f.invalid_transition_count,cumsum(double(invalid))) && ...
    all(f.invalid_transition_count==0),'Normal operation must have zero invalid transitions');
add('Integer signed count',all(f.decoded_encoder_count==fix(f.decoded_encoder_count)) && ...
    all(abs(f.decoded_encoder_count)<2^53),'int64 position, exactly representable at double estimator boundary');
edgeCount=f.ideal_count_edge; phi=f.encoder_edge_angle_rad*(p5.ENCODER_CPR/(2*pi));
boundary=abs(phi-round(phi))<1e-7;
expectedA=double(b.A_table(mod(edgeCount,4)+1));
expectedB=double(b.B_table(mod(edgeCount,4)+1));
add('A/B generator phase and angle scaling',all(edgeCount==floor(phi) | ...
    (boundary & abs(edgeCount-floor(phi))<=1)) && ...
    isequal(A,expectedA(:)) && isequal(B,expectedB(:)), ...
    '1024 A/B periods and 4096 Gray transitions per revolution');
travel=abs(diff(phi));
add('No skipped encoder edges',all(abs(diff(edgeCount))<=1) && all(travel<1) && ...
    max(abs(d.rpm))<b.design_max_rpm, ...
    sprintf('Max angle advance %.5g counts/edge tick; covers aliases not caught by invalid flag',max(travel)));
add('Decoded count equals same-trajectory ideal edge count',isequal(f.decoded_encoder_count,edgeCount), ...
    'Exact after initial alignment; no cumulative +/-1 offset accepted');
fprintf('\n--- Phase 5B estimator snapshot diagnostics ---\n');

% Check all e-table columns for non-finite values.
for k = 1:width(e)
    name = e.Properties.VariableNames{k};
    x = e.(name);

    if isnumeric(x) || islogical(x)
        bad = find(~isfinite(double(x)));
        if ~isempty(bad)
            fprintf('%s: %d non-finite values; first at row %d\n', ...
                name,numel(bad),bad(1));
        end
    end
end

% Count snapshot mismatch.
badCount = find(e.encoder_count ~= e.decoded_encoder_count);
fprintf('encoder_count vs decoded_encoder_count mismatches: %d\n',numel(badCount));

if ~isempty(badCount)
    k = badCount(1);
    fprintf('First count mismatch at row %d\n',k);
    disp(e(max(1,k-3):min(height(e),k+3), ...
        {'time_s','encoder_count','decoded_encoder_count', ...
        'encoder_delta_count','ideal_encoder_count_phase5a'}));
end

% Delta-count mismatch.
expectedDelta = diff(e.decoded_encoder_count);
badDelta = find(e.encoder_delta_count(2:end) ~= expectedDelta);

fprintf('delta-count mismatches: %d\n',numel(badDelta));

if ~isempty(badDelta)
    k = badDelta(1)+1;
    fprintf('First delta mismatch at row %d\n',k);
end

% Phase 5A comparison mismatch.
localDifference = e.decoded_encoder_count-e.ideal_encoder_count_phase5a;
samplePhi = e.shaft_angle_rad*(p5.ENCODER_CPR/(2*pi));
sampleBoundary = abs(samplePhi-round(samplePhi))<1e-7;

badIdeal = find(~(localDifference==0 | ...
    (sampleBoundary & abs(localDifference)<=1)));

fprintf('Phase 5A comparison mismatches: %d\n',numel(badIdeal));

if ~isempty(badIdeal)
    k = badIdeal(1);
    fprintf('First Phase 5A mismatch at row %d\n',k);

    disp(e(max(1,k-3):min(height(e),k+3), ...
        {'time_s','shaft_angle_rad','decoded_encoder_count', ...
        'ideal_encoder_count_phase5a','count_difference_vs_phase5a'}));
end
add('Decoded count is estimator input',isequal(e.encoder_count,e.decoded_encoder_count) && ...
    isequal(e.encoder_delta_count(2:end),diff(e.decoded_encoder_count)), ...
    'Post-transition snapshot; topology verified independently');
localDifference=e.decoded_encoder_count-e.ideal_encoder_count_phase5a;
samplePhi=e.shaft_angle_rad*(p5.ENCODER_CPR/(2*pi));
sampleBoundary=abs(samplePhi-round(samplePhi))<1e-7;
add('Phase 5A ideal comparison log',isequal(e.count_difference_vs_phase5a,localDifference) && ...
    all(localDifference==0 | (sampleBoundary & abs(localDifference)<=1)), ...
    'Only floating-point floor choices at an exact sample boundary allow one count');
m.max_local_count_difference=max(abs(localDifference)); m.final_local_count_difference=localDifference(end);
m.rms_local_count_difference=sqrt(mean(localDifference.^2));
m.max_edge_count_difference=max(abs(f.decoded_encoder_count-edgeCount));
m.invalid_transition_count=f.invalid_transition_count(end);
m.max_counts_per_edge_tick=max(travel); m.Tedge_s=b.Tedge;
    function add(name,pass,detail)
        status='FAIL'; if pass,status='PASS';end
        checks(end+1)=struct('name',name,'status',status,'detail',detail);
        fprintf('[%s] %s: %s\n',status,name,detail);
    end
end
