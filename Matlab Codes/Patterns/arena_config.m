function varargout = arena_config(varargin)
% ARENA_CONFIG M-file for arena_config.fig
%      ARENA_CONFIG, by itself, creates a new ARENA_CONFIG or raises the existing
%      singleton*.
%
%      H = ARENA_CONFIG returns the handle to a new ARENA_CONFIG or the handle to
%      the existing singleton*.
%
%      ARENA_CONFIG('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ARENA_CONFIG.M with the given input arguments.
%
%      ARENA_CONFIG('Property','Value',...) creates a new ARENA_CONFIG or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before arena_config_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to arena_config_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help arena_config

% Last Modified by GUIDE v2.5 04-Nov-2009 16:30:33

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @arena_config_OpeningFcn, ...
                   'gui_OutputFcn',  @arena_config_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before arena_config is made visible.
function arena_config_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to arena_config (see VARARGIN)

% Choose default command line output for arena_config
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes arena_config wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = arena_config_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes when entered data in editable cell(s) in arenaCfg.
function arenaCfg_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to arenaCfg (see GCBO)
% eventdata  structure with the following fields (see UITABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)
curPath = pwd;
panel_control_paths;
cd(cfg_path);

%load cfgdata
cfgData = get(handles.arenaCfg, 'Data');
handles.cfgData = cfgData;

Data_to_write=[cfgData(:,2);cfgData(:,4);cfgData(:,6);cfgData(:,8);...
    cfgData(:,10);cfgData(:,12);cfgData(:,14);cfgData(:,16)];

%update arena.cfg
fid = fopen([cfg_path '\arena.cfg' ] , 'w');
fwrite(fid, Data_to_write,'uchar');
fclose(fid);

%return to the original path
cd(curPath);


% --- Executes on button press in saveCfg.
function saveCfg_Callback(hObject, eventdata, handles)
% hObject    handle to saveCfg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

curPath = pwd;
panel_control_paths;
cd(cfg_path);

%load cfgdata
cfgData = get(handles.arenaCfg, 'Data');
handles.cfgData = cfgData;

%save the arena configuration table data to a mat file
[FileName,PathName] = uiputfile('*.mat','Save the arena configuration file');
save(fullfile(PathName,FileName), 'cfgData');
Data_to_write=[cfgData(:,2);cfgData(:,4);cfgData(:,6);cfgData(:,8);...
    cfgData(:,10);cfgData(:,12);cfgData(:,14);cfgData(:,16)];

%update arena.cfg
fid = fopen([cfg_path '\arena.cfg' ] , 'w');
fwrite(fid, Data_to_write,'uchar');
fclose(fid);

%return to the original path
cd(curPath);


% --- Executes on button press in openCfg.
function openCfg_Callback(hObject, eventdata, handles)
% hObject    handle to openCfg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
curPath = pwd;
panel_control_paths;
cd(cfg_path)

%load configuraton table file
[FileName,PathName] = uigetfile('*.mat','Select an arena configuration file');
if (all(FileName ~= 0))
    load([PathName FileName]);
    set(handles.arenaCfg,'Data',cfgData);
    handles.cfgData = cfgData;
end

guidata(hObject, handles);
cd(curPath);

% --- Executes on button press in loadCfg.
function loadCfg_Callback(hObject, eventdata, handles)
% hObject    handle to loadCfg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
curPath = pwd;
panel_control_paths;
cd(cfg_path)

% step 1 - get the drive letter
SD_drive = get_SD_drive;
if (SD_drive ~= -1)
    % step 2 - use dd.exe to burn the just created image - temp\SD.img
    dos(['copy /Y ' 'arena.cfg ' SD_drive ':\']);
else
    errordlg('Bad choice of disk, or disk is missing, try again');
end

cd(curPath);