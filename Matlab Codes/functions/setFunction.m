function varargout = setFunction(varargin)
% SETFUNCTION M-file for setFunction.fig
%      SETFUNCTION, by itself, creates a new SETFUNCTION or raises the existing
%      singleton*.
%
%      H = SETFUNCTION returns the handle to a new SETFUNCTION or the handle to
%      the existing singleton*.
%
%      SETFUNCTION('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SETFUNCTION.M with the given input arguments.
%
%      SETFUNCTION('Property','Value',...) creates a new SETFUNCTION or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before setFunction_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to setFunction_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help setFunction

% Last Modified by GUIDE v2.5 21-Apr-2011 08:30:51
% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @setFunction_OpeningFcn, ...
                   'gui_OutputFcn',  @setFunction_OutputFcn, ...
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


% --- Executes just before setFunction is made visible.
function setFunction_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to setFunction (see VARARGIN)
global SD currentState;
currentcurrentState.closeSetFun = 0; % a flag to check whether users close the GUI
currentState.setFun = 0; % a flag to check whether users click the send config button
% Choose default command line output for setFunction
handles.output = hObject;
funcList = SD.function.velFunctionName;
funcList = ['default function', funcList];
set(handles.setVelFuncX, 'String', funcList);
set(handles.setVelFuncX, 'Value', currentState.velFuncX);
set(handles.setVelFuncY, 'String', funcList);
set(handles.setVelFuncY, 'Value', currentState.velFuncY);
set(handles.funcXUpdateRate, 'String',num2str(currentState.funcXUpdateRate));
set(handles.funcYUpdateRate, 'String',num2str(currentState.funcYUpdateRate));
guidata(hObject, handles);

% UIWAIT makes setFunction wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = setFunction_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in setVelFuncX.
function setVelFuncX_Callback(hObject, eventdata, handles)
% hObject    handle to setVelFuncX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns setVelFuncX contents as cell array
%        contents{get(hObject,'Value')} returns selected item from setVelFuncX
%The first one is the default
global currentState;
contents = get(hObject,'String');
currentState.velFuncX = get(hObject,'Value');
currentState.velFuncXName = contents{currentState.velFuncX};
currentState.funcXName =  currentState.velFuncXName;


% --- Executes during object creation, after setting all properties.
function setVelFuncX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to setVelFuncX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function funcXUpdateRate_Callback(hObject, eventdata, handles)
% hObject    handle to funcXUpdateRate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of funcXUpdateRate as text
%        str2double(get(hObject,'String')) returns contents of funcXUpdateRate as a double  
% do some error checking
global currentState;
funcXUpdateRate = str2double(get(hObject,'String'));
currentState.funcXUpdateRate = funcXUpdateRate;
if ~(~isnan(funcXUpdateRate) && (funcXUpdateRate == round(funcXUpdateRate)) && (funcXUpdateRate > 0) && (funcXUpdateRate <= 500) )
    errordlg(['function X update rate must be a positive integer, not greater than 500 -- no action taken'],'Bad Input','modal')
end

% --- Executes during object creation, after setting all properties.
function funcXUpdateRate_CreateFcn(hObject, eventdata, handles)
% hObject    handle to funcXUpdateRate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in setVelFuncY.
function setVelFuncY_Callback(hObject, eventdata, handles)
% hObject    handle to setVelFuncY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns setVelFuncY contents as cell array
%        contents{get(hObject,'Value')} returns selected item from setVelFuncY
global currentState;
contents = get(hObject,'String');
currentState.velFuncY = get(hObject,'Value');
currentState.velFuncYName = contents{currentState.velFuncY};
currentState.funcYName = currentState.velFuncYName;



% --- Executes during object creation, after setting all properties.
function setVelFuncY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to setVelFuncY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in showFuncX.
function showFuncX_Callback(hObject, eventdata, handles)
% hObject    handle to showFuncX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global currentState;
display_function(currentState.velFuncXName);

% --- Executes on button press in showFuncY.
function showFuncY_Callback(hObject, eventdata, handles)
% hObject    handle to showFuncY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global currentState;
display_function(currentState.velFuncYName);

function funcYUpdateRate_Callback(hObject, eventdata, handles)
% hObject    handle to funcYUpdateRate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of funcYUpdateRate as text
%        str2double(get(hObject,'String')) returns contents of funcYUpdateRate as a double
global currentState;
funcYUpdateRate = str2double(get(hObject,'String')); 
currentState.funcYUpdateRate = funcYUpdateRate;
if ~( ~isnan(funcYUpdateRate) && (funcYUpdateRate == round(funcYUpdateRate)) && (funcYUpdateRate > 0) && (funcYUpdateRate <= 500) )
    errordlg(['function Y update rate must be a positive integer, not greater than 500 -- no action taken'],'Bad Input','modal')
end



% --- Executes during object creation, after setting all properties.
function funcYUpdateRate_CreateFcn(hObject, eventdata, handles)
% hObject    handle to funcYUpdateRate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in sendCfg.
function sendCfg_Callback(hObject, eventdata, handles)
% hObject    handle to sendCfg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global currentState newString;
catString = '';
Panel_com('set_velFunc_id', [1, currentState.velFuncX - 1]);  % call reset if OK
catString = newString;
Panel_com('set_funcX_freq', [currentState.funcXUpdateRate]);  
catString = [catString, newString];
Panel_com('set_velFunc_id', [2, currentState.velFuncY - 1;]);  % call reset if OK
catString = [catString, newString];
Panel_com('set_funcY_freq', [currentState.funcYUpdateRate]); 
catString = [catString, newString];
newString = catString;
currentState.closeSetFun = 1;
currentState.setFun = 1;
close(gcf);

function display_function(functionName)
load('Pcontrol_paths.mat');
funcFile = fullfile(function_path,functionName);
if ~exist(funcFile)
    [filename, pathname] = uigetfile('function*.mat', 'Pick an velocity function file');
    if ~filename
        return;
    else
        funcFile = fullFile(pathname, filename);
    end
end

load(funcFile)
load(funcFile);
time = 1:length(func);
scaled_func = round(20.*func);   % 20 = 1V
figure;
subplot(211)
plot(time, scaled_func);
% title('20 seconds of the function')
%xlabel('time')
ylabel('20 * volts')
subplot(212)
zoom_time = [time(end-19:end) time(end)+time(1:20)];
zoom_func = [scaled_func(end-19:end) scaled_func(1:20)];
plot(zoom_time, zoom_func);
hold on
plot([time(end) time(end)], [min(zoom_func) max(zoom_func)], 'r')
axis tight
xlabel('Make sure the function is periodic-click to close the window')
[a,b,button] = ginput(1);
close(gcf)


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
global currentState;
currentState.closeSetFun = 1;
delete(hObject);
