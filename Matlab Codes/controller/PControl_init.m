function PC = PControl_init()
%PControl_init
%PControl initializer script
global serialPort SD myPCCfg;
load('Pcontrol_paths.mat');

PC.x_gain_max = 10;
PC.x_gain_min = -10;
PC.x_gain_val = 0;

PC.y_gain_max = 10;
PC.y_gain_min = -10;
PC.y_gain_val = 0;

PC.x_offset_max = 5;
PC.x_offset_min = -5;
PC.x_offset_val = 0;

PC.y_offset_max = 5;
PC.y_offset_min = -5;
PC.y_offset_val = 0;

PC.x_pos = 1;
PC.y_pos = 1;


PC.x_mode = 0;  % default is open loop
PC.y_mode = 0;

PC.current_pattern = 0; % use this as the init value to check if real pattern is set

% load the SD file, we cannot load and save mat file in the init_serial
% because it is nested function and panel_control_paths.m cannnot be called.
if exist('myPCCfg.mat','file')
    load([controller_path '\myPCCfg'],'-mat');
else
    myPCCfg.portNum = 99; %trust user instead of a default value   
end

%if initialize serial port 
if init_serial == 1  
    Panel_com('ctr_reset');
end

%save serial port number to SD
save([controller_path '\myPCCfg'], 'myPCCfg');

%empty serialport input buffer
%This is necessary because uncleanned data in the buffer will be save to
%the SD.mat and corrupt the mat file.
while serialPort.BytesAvailable ~= 0
    messRevd = fscanf(serialPort);
end
%sync the sd information to the PC, save as SD.mat in controller_path
%Panel_com('sync_sd_info');

% load the SD file - 
load('Pcontrol_paths.mat');

if exist('SD.mat','file')
    try
        load([controller_path '\SD'])
        PC.num_patterns = SD.pattern.num_patterns;
        %PC.numVelFunc = SD.function.numVelFunc;
        %PC.numPosFunc = SD.function.numPosFunc;
        for j = 1:SD.pattern.num_patterns
            PC.pattern_x_size(j) = SD.pattern.x_num(j);
            PC.pattern_y_size(j) = SD.pattern.y_num(j);
        end
    catch ME
        SD.pattern.num_patterns=0;
        warndlg('The SD.mat file on the PC is corrupted by sync_ds_info command, you have to load pattern, fucntion, and/or config files to SD card again.','corrupted SD.mat file');
    end
        
else  % first time to ran PControl
    SD.pattern.num_patterns=0;
    warndlg('No SD.mat file is found on your PC, you can either use sync_sd_info command to load the SC.mat from SD card or load pattern, fucntion, and/or config files to a empty SD card.', 'No SD.mat found');
end

PC.num_patterns = SD.pattern.num_patterns;
%PC.numVelFunc = SD.function.numVelFunc;
%PC.numPosFunc = SD.function.numPosFunc;
for j = 1:SD.pattern.num_patterns
     PC.pattern_x_size(j) = SD.pattern.x_num(j); 
     PC.pattern_y_size(j) = SD.pattern.y_num(j); 
end

save([controller_path '\SD'], 'SD');

