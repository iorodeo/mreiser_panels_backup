%set patten id first
Panel_com('set_pattern_id', 1);

%create analog input channel and logging mode, NI-USB6009
ai = analoginput('nidaq', 'Dev6');
 
%AI0 and AI4 are the postive and negative inputs of differential analog input channel 0
chans = addchannel(ai, 0); 
set(ai, 'SampleRate', 10000);
set(ai,'LoggingMode','Disk&Memory');
set(ai,'LogFileName','c:\temp\jitter10.daq');
set(ai,'SamplesPerTrigger', inf);

start(ai);
pause(1);
%for gs = 1 (rc=0), benchmark rate is about 200 frame per second
for i = 10:10:120
    i
    Panel_com('send_gain_bias',[i,0,0,0]);
    Panel_com('start');
    pause(10);
    Panel_com('stop');
    pause(1);
end

stop(ai);
        