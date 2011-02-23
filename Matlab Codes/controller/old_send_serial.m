function send_serial(string_to_send)

% this is a function for sending serial port info from matlab. 
% it is written as an alternative to mex_serial
% written in a hurry by MBR, should be modifed to include error checking,
% etc., but seems to work as is, and has not crashed or hung. 

% fid = fopen('com8:', 'w');
% fwrite(fid, string_to_send, 'uchar');
% fclose(fid);
guidata(hObject, handles);
fprintf(handles.PC.serialPort, string_to_send);
