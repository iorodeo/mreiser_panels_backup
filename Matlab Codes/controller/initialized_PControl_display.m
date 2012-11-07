function initialized_PControl_display( )
% This function initialized the PControl GUI and function ID once a
% hardware reset is caught

global hPcontrol currentState;
if ~isempty(hPcontrol)
    handles = guihandles(hPcontrol);
    %initialization
    currentState.funcXUpdateRate = 50;
    currentState.funcYUpdateRate = 50;
    currentState.velFuncX = 1;
    currentState.velFuncY = 1;
    currentState.posFuncX = 1;
    currentState.posFuncY = 1;
    currentState.pattID = 1;
    
    currentState.velFuncXName = 'default';
    currentState.velFuncXName = 'default';
    currentState.posFuncXName = 'default';
    currentState.posFuncYName = 'default';
    currentState.funcXName = 'default';
    currentState.funcYName = 'default';
    currentState.pattName = 'default';
    
    set(handles.x_offset_slider, 'enable', 'off');
    set(handles.x_gain_slider, 'enable', 'off');
    set(handles.x_gain_zero, 'enable', 'off');
    set(handles.x_offset_zero, 'enable', 'off');
    set(handles.x_loop_menu, 'enable', 'off');
    set(handles.x_pos_val, 'enable', 'off');
    set(handles.x_pos_plus, 'enable', 'off');
    set(handles.x_pos_minus, 'enable', 'off');
    
    set(handles.y_offset_slider, 'enable', 'off');
    set(handles.y_gain_slider, 'enable', 'off');
    set(handles.y_gain_zero, 'enable', 'off');
    set(handles.y_offset_zero, 'enable', 'off');
    set(handles.y_loop_menu, 'enable', 'off');
    set(handles.y_pos_val, 'enable', 'off');
    set(handles.y_pos_plus, 'enable', 'off');
    set(handles.y_pos_minus, 'enable', 'off');
    
    set(handles.Start_button, 'enable', 'off');
    
    update_status_display('GUI info has been updated.');
end
