function update_status_display(new_string)
% this function updates the status display box at the bottom of the GUI

global hPcontrol;
if isempty(hPcontrol)
    Panel_com('quiet_mode_on');
    %disp(new_string);
else
    handles = guihandles(hPcontrol);
    rowNum = size(new_string, 1);
    buffer_length = 18;   % number of lines of text
    max_string_length = 60;   % max length for each string
    
    cut_length = max_string_length;
    
    temp_cell_array = get(handles.status_display, 'String');
    
    disp(new_string);
    for i = 1:rowNum
        if (i == 1)
            temp_cell_array{end+1} = [' > ' new_string];
        else
            temp_cell_array{end+1} = new_string;
        end
    end
    
    if (length(temp_cell_array) > buffer_length)
        set(handles.status_display, 'String', temp_cell_array(end-buffer_length+1:end));
    else
        set(handles.status_display, 'String', temp_cell_array);
    end

end
