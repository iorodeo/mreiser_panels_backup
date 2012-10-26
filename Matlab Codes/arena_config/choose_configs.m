function varargout = choose_configs(varargin)
% Edit the above text to modify the response to help choose_configs
% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @choose_configs_OpeningFcn, ...
                   'gui_OutputFcn',  @choose_configs_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin & isstr(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before choose_configs is made visible.
function choose_configs_OpeningFcn(hObject, eventdata, handles, varargin)

% Choose default command line output for choose_configs
handles.output = hObject;

%perform initializations here:
handles.file = [];
handles.file_index = 0;
set(handles.button_remove_file, 'enable', 'Off');
set(handles.button_loadcfg, 'enable', 'Off');
guidata(hObject, handles);  % Update handles structure

% --- Outputs from this function are returned to the command line.
function varargout = choose_configs_OutputFcn(hObject, eventdata, handles)
% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% do some backing up here
% save ....
selection = questdlg('Close Configuration Selection Tool?','Close Request Function', 'Yes','No','Yes');
switch selection,
   case 'Yes',
     delete(hObject)
   case 'No'
     return
end

% --- Executes during object creation, after setting all properties.
function listbox_files_CreateFcn(hObject, eventdata, handles)
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

% --- Executes on selection change in listbox_files.
function listbox_files_Callback(hObject, eventdata, handles)
% show the parameters for the highlighted entry
% set this to the file_index, so if changes are made to param list, can
% record them for the correct file
handles.file_index = get(handles.listbox_files,'Value');
guidata(hObject, handles);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%     Callbacks for all buttons goes here                           %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function button_add_file_Callback(hObject, eventdata, handles)
% --- Executes on button press in button_add_file.
curPath = pwd;
load('Pcontrol_paths.mat');
cd(cfg_path);

[FileName,PathName] = uigetfile('cfg*.mat','Select a configuration file');
% returns a 0 if cancel is pressed
if ~(isequal(FileName, 0)|isequal(PathName, 0))
    handles.file_index = length(handles.file) +1;
    handles.file(handles.file_index).FileName = FileName;
    handles.file(handles.file_index).PathName = PathName;
    handles.file_list{handles.file_index} = FileName;
    set(handles.listbox_files, 'String', handles.file_list);
    guidata(hObject, handles);
end
if(length(handles.file) > 0)
    set(handles.button_remove_file, 'enable', 'On');
    set(handles.button_loadcfg, 'enable', 'On');
end
cd(curPath);


function button_add_folder_Callback(hObject, eventdata, handles)
% --- Executes on button press in button_add_folder.

load('Pcontrol_paths.mat');
dir_path = uigetdir(cfg_path);
if ~isequal(dir_path, 0)
    dir_struct = dir(fullfile(dir_path, 'cfg*.mat'));
    [sorted_names,sorted_index] = sortrows({dir_struct.name}');
	% loop through the files in the folder, add one at a time
	for j = 1:length(sorted_names)
        handles.file_index = length(handles.file) + 1;
        handles.file(handles.file_index).FileName = sorted_names{j}; % use curly braces to index into cell array
        handles.file(handles.file_index).PathName = [dir_path filesep];
	    handles.file_list{handles.file_index} = sorted_names{j};
        set(handles.listbox_files, 'String', handles.file_list);
        guidata(hObject, handles);  % Update handles structure
    end    
end   
if(length(handles.file) > 0)
    set(handles.button_remove_file, 'enable', 'On');
    set(handles.button_loadcfg, 'enable', 'On');
end


% --- Executes on button press in button_remove_file.
function button_remove_file_Callback(hObject, eventdata, handles)
% removes the highlighted file from the list
handles.file_index = get(handles.listbox_files,'Value');
file_index = handles.file_index;
num_files = length(handles.file);
files_to_keep = [1:(file_index-1) (file_index+1):num_files];
% create an index of the files to keep    
handles.file = handles.file([files_to_keep]);    
rem_val = get(handles.listbox_files,'Value');
handles.file_list(rem_val) = [];
set(handles.listbox_files, 'String', handles.file_list);
set(handles.listbox_files,'Value', 1); % set this to 1 to prevent attempt to delete
handles.file_index = 1; 
% files just removed from the list
guidata(hObject, handles);  % Update handles structure
if(length(handles.file) <= 0)
    set(handles.button_remove_file, 'enable', 'Off');
    set(handles.button_loadcfg, 'enable', 'Off');
end


% --- Executes on button press in button_loadcfg.
function button_loadcfg_Callback(hObject, eventdata, handles)
global SD

curPath = pwd;
load('Pcontrol_paths.mat');
cd(cfg_path);
handles.SD = make_config_image(handles.file);
% step 1 - get the drive letter
SD_drive = get_SD_drive;
if (SD_drive ~= -1)
    if length(dir([SD_drive, ':\*.cfg']))
        dos(['del ' SD_drive ':\*.cfg']);
    end
    
    result1 = dos(['copy /Y ' temp_path '\*.cfg ' SD_drive ':\']);
    
    % In some card reader, sd_drive might be a removeable drive, but it is wrong one
    % give user a chance to input the correct drive
    if result1
        SD_drive = userInputSDDrive;
        
        if length(dir([SD_drive, ':\*.cfg']))
            dos(['del ' SD_drive ':\*.cfg']);
        end
        
        result2 = dos(['copy /Y ' temp_path '\*.cfg ' SD_drive ':\']);
        
        if result2
            disp('The SD_drive is a invalid drive!');
            return;
        end
    end
     
    SD.arenaConfig = handles.SD;
    save([controller_path '\SD'], 'SD');
    save([SD_drive ':\SD'], 'SD');
else
    errordlg('Bad choice of disk, or disk is missing, try again');
end

cd(curPath);
