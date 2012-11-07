function varargout = setPattern(varargin)
% SETAPATTERN M-file for setAPattern.fig
%      SETAPATTERN, by itself, creates a new SETAPATTERN or raises the existing
%      singleton*.
%
%      H = SETAPATTERN returns the handle to a new SETAPATTERN or the handle to
%      the existing singleton*.
%
%      SETAPATTERN('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SETAPATTERN.M with the given input arguments.
%
%      SETAPATTERN('Property','Value',...) creates a new SETAPATTERN or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before setPattern_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to setPattern_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help setAPattern

% Last Modified by GUIDE v2.5 24-Jul-2012 11:14:44
% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @setPattern_OpeningFcn, ...
                   'gui_OutputFcn',  @setPattern_OutputFcn, ...
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


% --- Executes just before setAPattern is made visible.
function setPattern_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to setAPattern (see VARARGIN)
global SD currentState;
% Choose default command line output for setAPattern


handles.output = hObject;

patternList = SD.pattern.pattNames;
set(handles.setAPattern, 'String', patternList);
set(handles.setAPattern, 'Value', currentState.pattID);
currentState.pattName = SD.pattern.pattNames{1};
currentState.chosePat = 0; % a flag to judge whether user chooses a pattern
currentState.closeSetPat = 0; % a flag to check whether user closes the GUI

guidata(hObject, handles);

% UIWAIT makes setAPattern wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = setPattern_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in setAPattern.
function setAPattern_Callback(hObject, eventdata, handles)
% hObject    handle to setAPattern (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns setAPattern contents as cell array
%        contents{get(hObject,'Value')} returns selected item from setAPattern
%The first one is the default


global currentState
contents = get(hObject,'String');
currentState.pattID = get(hObject,'Value');
currentState.pattName = contents{currentState.pattID};
pattID = currentState.pattID;




% --- Executes during object creation, after setting all properties.
function setAPattern_CreateFcn(hObject, eventdata, handles)
% hObject    handle to setAPattern (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in loadPattern.
function loadPattern_Callback(hObject, eventdata, handles)
% hObject    handle to loadPattern (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global currentState SD;
if SD.pattern.num_patterns ~=0
    currentState.chosePat =1;
    currentState.closeSetPat = 1;
    pattID = currentState.pattID;
    Panel_com('set_pattern_id', [pattID]);
    close(gcf);
end
    

% --- Executes on button press in showPatt.
function showPatt_Callback(hObject, eventdata, handles)
% hObject    handle to showPatt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global currentState;
load('Pcontrol_paths.mat');
pattFullName = fullfile(pattern_path, currentState.pattName);
if exist(pattFullName)
    play_current_pattern;
else
    Pattern_Player;
end


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
global currentState;
currentState.closeSetPat = 1;
delete(hObject);


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over setAPattern.
function setAPattern_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to setAPattern (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in loadPattern2Panels.
function loadPattern2Panels_Callback(hObject, eventdata, handles)
% hObject    handle to loadPattern2Panels (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global currentState SD;
if SD.pattern.num_patterns ~=0
    currentState.chosePat =1;
    currentState.closeSetPat = 1;
    pattID = currentState.pattID;
    Panel_com('load_pattern_2panels', [pattID]);
    close(gcf);
end