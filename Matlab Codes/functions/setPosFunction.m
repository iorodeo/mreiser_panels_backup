function varargout = setPosFunction(varargin)
% SETPOSFUNCTION M-file for setPosFunction.fig
%      SETPOSFUNCTION, by itself, creates a new SETPOSFUNCTION or raises the existing
%      singleton*.
%
%      H = SETPOSFUNCTION returns the handle to a new SETPOSFUNCTION or the handle to
%      the existing singleton*.
%
%      SETPOSFUNCTION('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SETPOSFUNCTION.M with the given input arguments.
%
%      SETPOSFUNCTION('Property','Value',...) creates a new SETPOSFUNCTION or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before setPosFunction_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to setPosFunction_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help setPosFunction

% Last Modified by GUIDE v2.5 17-Dec-2009 16:07:49

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @setPosFunction_OpeningFcn, ...
                   'gui_OutputFcn',  @setPosFunction_OutputFcn, ...
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


% --- Executes just before setPosFunction is made visible.
function setPosFunction_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to setPosFunction (see VARARGIN)
global SD currentState;
currentState.closeSetPosFun = 0; % a flag to check whether users close the GUI
currentState.setPosFun = 0; % a flag to check whether users click the send config button

% Choose default command line output for setPosFunction
handles.output = hObject;
funcList = SD.function.posFunctionName;
funcList = ['default function', funcList];
set(handles.setPosFuncX, 'String', funcList);
set(handles.setPosFuncX, 'Value', currentState.posFuncX);
set(handles.setPosFuncY, 'String', funcList);
set(handles.setPosFuncY, 'Value', currentState.posFuncY);
set(handles.funcXUpdateRate, 'String',num2str(currentState.funcXUpdateRate));
set(handles.funcYUpdateRate, 'String',num2str(currentState.funcYUpdateRate));
guidata(hObject, handles);

% UIWAIT makes setPosFunction wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = setPosFunction_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in setPosFuncX.
function setPosFuncX_Callback(hObject, eventdata, handles)
% hObject    handle to setPosFuncX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns setPosFuncX contents as cell array
%        contents{get(hObject,'Value')} returns selected item from setPosFuncX
global currentState;
contents = get(hObject,'String');
currentState.posFuncX = get(hObject,'Value');
currentState.posFuncXName = contents{currentState.posFuncX};
currentState.funcXName = currentState.posFuncXName;


% --- Executes during object creation, after setting all properties.
function setPosFuncX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to setPosFuncX (see GCBO)
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
%        str2double(get(hObject,'String')) returns contents of
%        funcXUpdateRate as a double
   
% do some error checking
global currentState;
funcXUpdateRate = str2double(get(hObject,'String'));  
currentState.funcXUpdateRate = funcXUpdateRate;
if ~( ~isnan(funcXUpdateRate) && (funcXUpdateRate == round(funcXUpdateRate)) && (funcXUpdateRate > 0) && (funcXUpdateRate <= 500) )
    errordlg(['Position function X update rate must be a positive integer, not greater than 500 -- no action taken'],'Bad Input','modal')
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


% --- Executes on selection change in setPosFuncY.
function setPosFuncY_Callback(hObject, eventdata, handles)
% hObject    handle to setPosFuncY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns setPosFuncY contents as cell array
%        contents{get(hObject,'Value')} returns selected item from setPosFuncY
global currentState;
contents = get(hObject,'String');
currentState.posFuncY = get(hObject,'Value');
currentState.posFuncYName = contents{currentState.posFuncY};
currentState.funcYName = currentState.posFuncYName;


% --- Executes during object creation, after setting all properties.
function setPosFuncY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to setPosFuncY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



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
    errordlg(['Position function Y update rate must be a positive integer, not greater than 500 -- no action taken'],'Bad Input','modal')
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
Panel_com('set_posFunc_id', [1, currentState.posFuncX - 1]);  % call reset if OK
catString = newString;
Panel_com('set_funcX_freq', [currentState.funcXUpdateRate]);  
catString = [catString, newString];
Panel_com('set_posFunc_id', [2, currentState.posFuncY - 1;]);  % call reset if OK
catString = [catString, newString];
Panel_com('set_funcY_freq', [currentState.funcYUpdateRate]); 
catString = [catString, newString];
newString = catString;
currentState.closeSetPosFun = 1;
currentState.setPosFun = 1;
close(gcf);


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
global currentState;
currentState.closeSetPosFun = 1;
delete(hObject);

