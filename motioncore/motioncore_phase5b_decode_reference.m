function [count,step,invalid,previous] = motioncore_phase5b_decode_reference(state,initialState,initialCount)
% Independent arithmetic oracle; does not consume the decoder's 16-entry LUT.
state=double(state(:)); assert(all(ismember(state,0:3)));
previous=[double(initialState);state(1:end-1)];
grayPhase=[0 1 3 2]; % binary-state index -> position within forward Gray cycle
phase=grayPhase(state+1); oldPhase=grayPhase(previous+1);
movement=mod(phase(:)-oldPhase(:),4);
step=zeros(size(state)); step(movement==1)=1; step(movement==3)=-1;
invalid=movement==2;
count=double(initialCount)+cumsum(step);
end
