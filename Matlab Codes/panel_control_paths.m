% set PControl paths
mfile_path = mfilename('fullpath');
[root_path, ~, ~] = fileparts(mfile_path);
function_path = strcat(root_path, filesep, 'functions');
pattern_path =  strcat(root_path, filesep, 'patterns');
controller_path = strcat(root_path, filesep, 'controller');
cfg_path = strcat(root_path, filesep, 'arena_config');

% Create temporary directory if it doesn't exist
home_path = sprintf('%s%s',getenv('HOMEDRIVE'),getenv('HOMEPATH'));
pcontrol_home_path = strcat(home_path, filesep, 'pcontrol');
if ~exist(pcontrol_home_path, 'dir')
    [flag, msg, ~] = mkdir(home_path, 'pcontrol');
    if flag ~= true
        error('unable to create directory: %s, %s', home_path_pcontrol, msg);
    end
end
temp_path = strcat(pcontrol_home_path, filesep, 'temp');
if ~exist(temp_path, 'dir')
   [flag, msg, ~] = mkdir(pcontrol_home_path, 'temp');
   if flag ~= true
       error('unable to create directory: %s, %s', temp_path, msg);
   end
end


% function_path = 'D:\Michael_Reiser\XmegaController_Matlab\functions';
% pattern_path =  'D:\Michael_Reiser\XmegaController_Matlab\patterns';
% temp_path = 'C:\temp';
% root_path = 'D:\Michael_Reiser\XmegaController_Matlab';
% controller_path = 'D:\Michael_Reiser\XmegaController_Matlab\controller';
% cfg_path = 'D:\Michael_Reiser\XmegaController_Matlab\arena_config';