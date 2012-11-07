[filename, pathname, filterindex] = uigetfile('*.mat', 'Pick an pattern Mat file');

filename = fullfile(pathname, filename);

load(filename);
%load('D:\Michael_Reiser\XmegaController_Matlab\Patterns\Pattern_4x4_blocks_48.mat');
pattern.row_compression = 0;

if pattern.row_compression == 1
    frame_length = pattern.gs_val * pattern.num_panels;
else
    frame_length = pattern.gs_val * pattern.num_panels * 8;
end
    
t= timer;
counter = 1;
set(t,'ExecutionMode','fixedrate');
frame_num = 0;
set(t, 'timerFcn',['tstart = tic;',...
                   'data = transpose(pattern.data(frame_num*frame_length+1:(frame_num + 1)*frame_length));',...
                   'Panel_com(''dump_frame'', [frame_length, frame_num/pattern.x_num * 2047, 0, pattern.num_panels,  pattern.gs_val, pattern.row_compression, data]);',...I
                   'frame_num = mod(frame_num + 1, pattern.x_num);',...
                   'counter = counter + 1;',...
                   'etime{counter}=toc(tstart);']);

set(t,'tasksToExecute', 1000);

i=50;
set(t,'Period',1/i);
ts=tic;
start(t);
pause(1000/i +5);
stop(t);
telapsed=toc(ts)


