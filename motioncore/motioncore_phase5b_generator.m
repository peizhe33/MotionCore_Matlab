function motioncore_phase5b_generator(q)
% Behavioral stimulus model only. Only A/B connect to the decoder.
add_block('built-in/SubSystem',q);
port(q,'In1','Angle',1,[20 45 50 59]);
add_block('simulink/Discrete/Zero-Order Hold',[q '/Sample_Angle'], ...
    'SampleTime','b.Tedge','Position',[90 35 155 75]);
add_block('simulink/Math Operations/Gain',[q '/Counts_Per_Radian'], ...
    'Gain','p5.ENCODER_CPR/(2*pi)','Position',[190 35 270 75]);
add_block('simulink/Math Operations/Rounding Function',[q '/Edge_Position'], ...
    'Operator','floor','Position',[315 35 380 75]);
add_block('simulink/Math Operations/Math Function',[q '/Phase_Mod4'], ...
    'Operator','mod','Position',[430 35 475 85]);
add_block('simulink/Sources/Constant',[q '/Four'],'Value','4','Position',[330 115 375 145]);
add_block('simulink/Signal Attributes/Data Type Conversion',[q '/Phase_Index'], ...
    'OutDataTypeStr','uint8','Position',[520 40 580 80]);
lookup(q,'A_Level','b.A_table',[630 20 705 65]);
lookup(q,'B_Level','b.B_table',[630 95 705 140]);
port(q,'Out1','A',1,[800 33 830 47]); port(q,'Out1','B',2,[800 108 830 122]);
port(q,'Out1','Ideal_Edge_Count',3,[800 208 830 222]);
port(q,'Out1','Sampled_Angle',4,[800 288 830 302]);
wire(q,'Angle/1','Sample_Angle/1'); wire(q,'Sample_Angle/1','Counts_Per_Radian/1');
wire(q,'Counts_Per_Radian/1','Edge_Position/1'); wire(q,'Edge_Position/1','Phase_Mod4/1');
wire(q,'Four/1','Phase_Mod4/2'); wire(q,'Phase_Mod4/1','Phase_Index/1');
wire(q,'Phase_Index/1','A_Level/1'); wire(q,'Phase_Index/1','B_Level/1');
wire(q,'A_Level/1','A/1'); wire(q,'B_Level/1','B/1');
wire(q,'Edge_Position/1','Ideal_Edge_Count/1'); wire(q,'Sample_Angle/1','Sampled_Angle/1');
end
function lookup(q,n,table,pos)
add_block('simulink/Lookup Tables/Direct Lookup Table (n-D)',[q '/' n], ...
    'NumberOfTableDimensions','1','Table',table,'Position',pos);
end
function port(q,kind,n,number,pos)
add_block(['simulink/Ports & Subsystems/' kind],[q '/' n],'Port',num2str(number),'Position',pos);
end
function wire(q,s,d)
add_line(q,s,d,'autorouting','on');
end
