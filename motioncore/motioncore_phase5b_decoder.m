function motioncore_phase5b_decoder(q)
% Standard-block RTL-style decoder. Exposes POST-transition position count.
% An invalid diagonal transition increments diagnostic count, leaves position
% unchanged, and resynchronizes previous_state to current_state on this tick.
add_block('built-in/SubSystem',q);
port(q,'In1','A',1,[20 35 50 49]); port(q,'In1','B',2,[20 95 50 109]);
convert(q,'A_Code','uint8',[80 25 125 55]); convert(q,'B_Code','uint8',[80 85 125 115]);
gain(q,'A_Weight','uint8(2)',[150 25 200 55]);
sumblock(q,'State','++',[240 45 270 105],'uint8');
delay(q,'Previous_State','b.initial_state','uint8',[290 185 355 225]);
gain(q,'Previous_Weight','uint8(4)',[390 185 455 225]);
sumblock(q,'Transition_Index','++',[495 50 525 115],'uint8');
lookup(q,'Transition_Delta','b.delta_table',[565 45 680 90]);
lookup(q,'Invalid_Transition','b.invalid_table',[565 200 680 245]);
convert(q,'Signed_Step','int64',[715 45 780 85]);
sumblock(q,'Updated_Count','++',[835 40 865 110],'int64');
delay(q,'Position_Register','b.initial_count','int64',[805 150 905 195]);
convert(q,'Invalid_Increment','uint64',[715 250 780 290]);
sumblock(q,'Updated_Invalid_Count','++',[835 255 865 325],'uint64');
delay(q,'Invalid_Register','uint64(0)','uint64',[805 360 905 405]);
wire(q,'A/1','A_Code/1'); wire(q,'A_Code/1','A_Weight/1');
wire(q,'B/1','B_Code/1'); wire(q,'A_Weight/1','State/1'); wire(q,'B_Code/1','State/2');
wire(q,'State/1','Previous_State/1'); wire(q,'Previous_State/1','Previous_Weight/1');
wire(q,'Previous_Weight/1','Transition_Index/1'); wire(q,'State/1','Transition_Index/2');
wire(q,'Transition_Index/1','Transition_Delta/1'); wire(q,'Transition_Index/1','Invalid_Transition/1');
wire(q,'Transition_Delta/1','Signed_Step/1'); wire(q,'Signed_Step/1','Updated_Count/1');
wire(q,'Position_Register/1','Updated_Count/2'); wire(q,'Updated_Count/1','Position_Register/1');
wire(q,'Invalid_Transition/1','Invalid_Increment/1'); wire(q,'Invalid_Increment/1','Updated_Invalid_Count/1');
wire(q,'Invalid_Register/1','Updated_Invalid_Count/2'); wire(q,'Updated_Invalid_Count/1','Invalid_Register/1');
names={'Decoded_Count','State_Out','Previous_State_Out','Direction','Invalid_Count'};
sources={'Updated_Count/1','State/1','Previous_State/1','Transition_Delta/1','Updated_Invalid_Count/1'};
for k=1:numel(names)
    port(q,'Out1',names{k},k,[1080 30+75*k 1110 44+75*k]); wire(q,sources{k},[names{k} '/1']);
end
end
function lookup(q,n,table,pos)
add_block('simulink/Lookup Tables/Direct Lookup Table (n-D)',[q '/' n], ...
    'NumberOfTableDimensions','1','Table',table,'Position',pos);
end
function delay(q,n,ic,type,pos) %#ok<INUSD>
add_block('simulink/Discrete/Unit Delay',[q '/' n], ...
    'SampleTime','b.Tedge', ...
    'InitialCondition',ic, ...
    'Position',pos);
end
function convert(q,n,type,pos)
add_block('simulink/Signal Attributes/Data Type Conversion',[q '/' n], ...
    'OutDataTypeStr',type,'Position',pos);
end
function gain(q,n,value,pos)
add_block('simulink/Math Operations/Gain',[q '/' n],'Gain',value,'Position',pos);
end
function sumblock(q,n,inputs,pos,type)
add_block('simulink/Math Operations/Sum',[q '/' n],'Inputs',inputs, ...
    'OutDataTypeStr',type,'SaturateOnIntegerOverflow','on','Position',pos);
end
function port(q,kind,n,number,pos)
add_block(['simulink/Ports & Subsystems/' kind],[q '/' n],'Port',num2str(number),'Position',pos);
end
function wire(q,s,d)
add_line(q,s,d,'autorouting','on');
end
