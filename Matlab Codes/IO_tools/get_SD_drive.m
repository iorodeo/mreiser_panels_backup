function [SD_drive] = get_SD_drive
% [SD_drive] = get_SD_drive
% function takes no inputs and returns the letter of the SD drive
% it also checks to see that the SD drive is indeed removable
global myPCCfg;

panel_control_paths;

if exist('myPCCfg.mat','file')
    load([controller_path '\myPCCfg'],'-mat');
    if isfield(myPCCfg, 'SDDrive')
        SD_drive = myPCCfg.SDDrive;
        myPCCfg.SDDrive = SD_drive;
        save([controller_path '\myPCCfg'], 'myPCCfg');
    else
        SD_drive = userInputSDDrive;
        myPCCfg.SDDrive = SD_drive;
        save([controller_path '\myPCCfg'], 'myPCCfg');
    end
else
    SD_drive = userInputSDDrive;
end

% this first line will need to be modified by the user to name the drive
% letter of the SD drive. This is very important!!! even though there is
% some error checking, if this is wrong, bad things may happen!

% check to make sure the drive is removable
% step 1 - issue an operating system command to get the drive type
[s, drive_type] = dos(['fsutil fsinfo drivetype ' SD_drive ':']);
% step 2 - see if drive type contains string - 'removable drive'
if (s == 0) && ~isempty(strfind(lower(drive_type), 'removable drive'));
    display(['SD drive selected is drive ' SD_drive ' which is a removable drive']);
else
    SD_drive = userInputSDDRive();
end