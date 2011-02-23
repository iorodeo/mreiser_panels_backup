function varargout = setConfig(varargin)
% SETACONFIG M-file for setAConfig.fig
%      SETACONFIG, by itself, creates a new SETACONFIG or raises the existing
%      singleton*.
%
%      H = SETACONFIG returns the handle to a new SETACONFIG or the handle to
%      the existing singleton*.
%
%      SETACONFIG('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SETACONFIG.M with the given input arguments.
%
%      SETACONFIG('Property','Value',...) creates a new SETACONFIG or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before setConfig_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to setConfig_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help setAConfig

% Last Modified by GUIDE v2.5 27-Oct-2010 17:32:15
% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @setConfig_OpeningFcn, ...
                   'gui_OutputFcn',  @setConfig_OutputFcn, ...
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


% --- Executes just before setAConfig is made visible.
function setConfig_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to setAConfig (see VARARGIN)
global SD currentState;
% Choose default command line output for setAConfig


handles.output = hObject;
configList = SD.arenaConfig.cfgNames;
set(handles.setAConfig, 'String', configList);
set(handles.setAConfig, 'Value', currentState.cfgID);
currentState.cfgName = SD.arenaConfig.cfgNames{1};
currentState.chosecfg = 0; % a flag to judge whether user chooses a pattern 
currentState.closeSetcfg = 0; % a flag to check whether user closes the GUI
guidata(hObject, handles);


% UIWAIT makes setAConfig wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = setConfig_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on selection change in setAConfig.
function setAConfig_Callback(hObject, eventdata, handles)
% hObject    handle to setAConfig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns setAConfig contents as cell array
%        contents{get(hObject,'Value')} returns selected item from setAConfig
%The first one is the default


global currentState
contents = get(hObject,'String');
currentState.cfgID = get(hObject,'Value');
currentState.cfgName = contents{currentState.cfgID};
cfgID = currentState.cfgID;




% --- Executes during object creation, after setting all properties.
function setAConfig_CreateFcn(hObject, eventdata, handles)
% hObject    handle to setAConfig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in loadConfig.
function loadConfig_Callback(hObject, eventdata, handles)
% hObject    handle to loadConfig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global currentState;
currentState.chosecfg =1;
currentState.closeSetcfg = 1;
cfgID = currentState.cfgID;
Panel_com('set_config_id', [cfgID]); 
close(gcf);


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
global currentState;
currentState.closeSetcfg = 1;
delete(hObject);

