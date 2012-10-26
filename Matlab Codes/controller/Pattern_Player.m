function varargout = Pattern_Player(varargin)
% this is a gui for playing patterns
% updated on 03.10.04 by MBR to allow for displaying gray scale images

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Pattern_Player_OpeningFcn, ...
                   'gui_OutputFcn',  @Pattern_Player_OutputFcn, ...
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


% --- Executes just before Pattern_Player is made visible.
function Pattern_Player_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Pattern_Player (see VARARGIN)

[I, map] = imread('Please_Load.bmp', 'BMP');
axes(handles.axes1); 
colormap(map);
image(I);
axis off; axis image; 

handles.output = hObject;
handles.x_rate = 0;
handles.y_rate = 0;
handles.x_pos = 1;
handles.y_pos = 1;
handles.pattern_x_size = 1;
handles.pattern_y_size = 1;
handles.Playing = 0;

%for smooth graphics
set(gcf,'DoubleBuffer','on');

% Update handles structure
guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = Pattern_Player_OutputFcn(hObject, eventdata, handles)
varargout{1} = handles.output;


% --- Executes during object creation, after setting all properties.
function x_pos_val_CreateFcn(hObject, eventdata, handles)
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


function x_pos_val_Callback(hObject, eventdata, handles)
user_entry = str2double(get(hObject,'string'));
if isnan(user_entry)
    errordlg('You must enter a numeric value','Bad Input','modal')
    set(handles.x_pos_val, 'string', num2str(handles.x_pos));
elseif (user_entry ~= round(user_entry) )
    errordlg('You must enter an integer','Bad Input','modal')
    set(handles.x_pos_val, 'string', num2str(handles.x_pos));
elseif ( (user_entry < 0)|(user_entry > handles.pattern_x_size) )
    errordlg('Number is out of the range for this pattern','Bad Input','modal')
    set(handles.x_pos_val, 'string', num2str(handles.x_pos));
else  % once you get here this is actually good input
    handles.x_pos = user_entry;
    guidata(hObject, handles);
end
display_curr_frame(handles)


% --- Executes on button press in x_pos_plus.
function x_pos_plus_Callback(hObject, eventdata, handles)
% increment the x_pos, wrap around if too big
temp_pos = handles.x_pos + 1;
if (temp_pos > handles.pattern_x_size)
    temp_pos = 1;
end
handles.x_pos = temp_pos;
set(handles.x_pos_val, 'string', num2str(temp_pos));
guidata(hObject, handles);
display_curr_frame(handles)


% --- Executes on button press in x_pos_minus.
function x_pos_minus_Callback(hObject, eventdata, handles)
% decrement the x_pos, wrap around if hits zero
temp_pos = handles.x_pos - 1;
if (temp_pos <= 0) 
    temp_pos = handles.pattern_x_size;
end
handles.x_pos = temp_pos;
set(handles.x_pos_val, 'string', num2str(temp_pos));
guidata(hObject, handles);
display_curr_frame(handles)


% --- Executes during object creation, after setting all properties.
function y_pos_val_CreateFcn(hObject, eventdata, handles)

if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


function y_pos_val_Callback(hObject, eventdata, handles)
user_entry = str2double(get(hObject,'string'));
if isnan(user_entry)
    errordlg('You must enter a numeric value','Bad Input','modal')
    set(handles.y_pos_val, 'string', num2str(handles.y_pos));
elseif (user_entry ~= round(user_entry) )
    errordlg('You must enter an integer','Bad Input','modal')
    set(handles.y_pos_val, 'string', num2str(handles.y_pos));
elseif ( (user_entry < 0)|(user_entry > handles.pattern_y_size) )
    errordlg('Number is out of the range for this pattern','Bad Input','modal')
    set(handles.y_pos_val, 'string', num2str(handles.y_pos));
else  % once you get here this is actually good input
    handles.y_pos = user_entry;
    guidata(hObject, handles);
end
display_curr_frame(handles)


% --- Executes on button press in y_pos_plus.
function y_pos_plus_Callback(hObject, eventdata, handles)
% increment the y_pos, wrap around if too big
temp_pos = handles.y_pos + 1;
if (temp_pos > handles.pattern_y_size)
    temp_pos = 1;
end
handles.y_pos = temp_pos;
set(handles.y_pos_val, 'string', num2str(temp_pos));
guidata(hObject, handles);
display_curr_frame(handles)

% --- Executes on button press in y_pos_minus.
function y_pos_minus_Callback(hObject, eventdata, handles)
% decrement the y_pos, wrap around if hits zero
temp_pos = handles.y_pos - 1;
if (temp_pos <= 0) 
    temp_pos = handles.pattern_y_size;
end
handles.y_pos = temp_pos;
set(handles.y_pos_val, 'string', num2str(temp_pos));
guidata(hObject, handles);
display_curr_frame(handles)

% --------------------------------------------------------------------
function menu_load_pattern_Callback(hObject, eventdata, handles)
load('Pcontrol_paths.mat');
cd(pattern_path)
[FileName,PathName] = uigetfile('P*.mat','Select a Pattern File');
if (all(FileName ~= 0))
    load([PathName FileName]);
    handles.pattern = pattern;

    handles.pattern_x_size = pattern.x_num;
    handles.pattern_y_size = pattern.y_num;
    handles.x_pos = 1;
    set(handles.x_pos_val, 'string', num2str(handles.x_pos));
    set(handles.x_pos_val, 'enable', 'on');
    handles.y_pos = 1; 
    set(handles.y_pos_val, 'string', num2str(handles.y_pos));
    set(handles.y_pos_val, 'enable', 'on');
    set(handles.y_pos_plus, 'enable', 'on');
    set(handles.y_pos_minus, 'enable', 'on');
    set(handles.x_pos_plus, 'enable', 'on');
    set(handles.x_pos_minus, 'enable', 'on');
    
    set(handles.Pixel_check, 'enable', 'on');
    set(handles.Pixel_check, 'value', 0);
    set(handles.Address_check, 'enable', 'on');
    set(handles.Address_check, 'value', 0);
    set(handles.Panel_check, 'enable', 'on');
    set(handles.Panel_check, 'value', 0);
    
    guidata(hObject, handles); 
    cla
    display_curr_frame(handles);

end

function display_curr_frame(handles)
% the color maps
switch handles.pattern.gs_val
    case 1
        C = [0 0 0; 0 1 0];   % 2 colors - on / off
    case 2
        C = [0 0 0; 0 1/3 0; 0 2/3 0; 0 1 0]; % 4 levels of gscale    
    case 3
        C = [0 0 0; 0 2/8 0; 0 3/8 0; 0 4/8 0; 0 5/8 0; 0 6/8 0; 0 7/8 0; 0 1 0];  % 8 levels of gscale        
    case 4
        C = [0 0 0; 0 2/16 0; 0 3/16 0; 0 4/16 0; 0 5/16 0; 0 6/16 0; 0 7/16 0; 0 8/16 0; ...
            0 9/16 0; 0 10/16 0; 0 11/16 0; 0 12/16 0; 0 13/16 0; 0 14/16 0; 0 15/16 0; 0 1 0];  % 16 levels of gscale        
    otherwise
        error('the graycale value is not appropriately set for this pattern - must be 1, 2, 3, or 4');
end

axes(handles.axes1)
% here we add a one to the image to correctly index into the color map
%imshow(handles.pattern.Pats(:,:,handles.x_pos,handles.y_pos)+1, C, 'notruesize')

% place to adjust displayed image for row_compression - 
row_compression = 0;
if isfield(handles.pattern, 'row_compression') % for backward compatibility
    if (handles.pattern.row_compression)
        row_compression = 1;
    end
end
if row_compression % this is probably incorrect for multi-row row compressed pats.
    image(repmat(handles.pattern.Pats(:,:,handles.x_pos,handles.y_pos)+1, 8, 1));
else
    image(handles.pattern.Pats(:,:,handles.x_pos,handles.y_pos)+1);
end    
axis off; axis image; colormap(C);
hold on
[numR, numC] = size(handles.pattern.Panel_map);
numRows = numR*8;
numCols = numC*8;
% plot Pixel_lines
if (get(handles.Pixel_check,'Value') == get(handles.Pixel_check,'Max'))
    %make horizontal lines
    for j = 1.5:numRows
            plot([0.5 numCols + 0.5], [j j],'w');
    end
    %make vertical lines
    for j = 1.5:numCols
            plot([j j], [0.5 numRows + 0.5],'w');
    end
        % plot Panel_lines
end
    
if(get(handles.Panel_check,'Value') == get(handles.Panel_check,'Max'))
    for j = 1.5:numRows
        if (mod(j,8) == 0.5) plot([0.5 numCols + 0.5], [j j],'r', 'LineWidth',2);
        end
    end
    %make vertical lines
    for j = 1.5:numCols
        if (mod(j,8) == 0.5) plot([j j], [0.5 numRows + 0.5],'r', 'LineWidth',2);
        end
    end
end

if(get(handles.Address_check,'Value') == get(handles.Address_check,'Max'))
    for i = 1:numR
        for j = 1:numC
            if (handles.pattern.Panel_map(i,j)) >= 10 % 2 digits
                h = text((j-1)*8 + 2,(i-1)*8 + 4,num2str(handles.pattern.Panel_map(i,j)));
            else
                h = text((j-1)*8 + 4,(i-1)*8 + 4,num2str(handles.pattern.Panel_map(i,j)));
            end
            set(h, 'Color', 'm', 'FontSize', 20, 'FontWeight', 'bold');
        end
    end
end


% --- Executes on button press in Panel_check.
function Panel_check_Callback(hObject, eventdata, handles)
display_curr_frame(handles);


% --- Executes on button press in Pixel_check.
function Pixel_check_Callback(hObject, eventdata, handles)
display_curr_frame(handles);


% --- Executes on button press in Address_Check.
function Address_check_Callback(hObject, eventdata, handles)
display_curr_frame(handles);



