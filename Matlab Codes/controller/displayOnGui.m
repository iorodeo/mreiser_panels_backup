function displayOnGui(obj,event)

global hPcontrol
% Define error message.
error1 = 'Type ''help instrument\instrcallback'' for an example using INSTRCALLBACK.';
error1Id = 'MATLAB:instrument:instrcallback:invalidSyntax';

switch nargin
case 0
   error(error1Id, ['This function may not be called with 0 inputs.\n',...
         'Type ''help instrument\instrcallback'' for an example using INSTRCALLBACK.']);
case 1
   error(error1Id, error1);
case 2
   if ~isa(obj, 'instrument') || ~isa(event, 'struct')
      error(error1Id, error1);
   end   
   if ~(isfield(event, 'Type') && isfield(event, 'Data'))
      error(error1Id, error1);
   end
end  

% Determine the type of event.
EventType = event.Type;

% Determine the time of the error event.
EventData = event.Data;
EventDataTime = EventData.AbsTime;
   
% Create a display indicating the type of event, the time of the event and
% the name of the object.
% name = get(obj, 'Name');
% fprintf([EventType ' event occurred at ' datestr(EventDataTime,13),...
% 	' for the object: ' name '.\n']);

if ~isempty(hPcontrol)
    handles = guihandles(hPcontrol);
end

% Display the error string.
if strcmpi(EventType, 'error')
	newString = fprintf([EventData.Message '\n']);
end



if strcmpi(EventType, 'BytesAvailable')
        byteLen = obj.BytesAvailable;
        if byteLen >= 250  %make sure 250 is a good threshhold
            %synchronize the sd.mat file   
            pause(0.3);
            out = fread(obj,obj.BytesAvailable,'uint8');
            %double check whether the data are of the SD.mat or not
            if (out(1)==50)&&out(out(52)==50)&&(out(103)==50)&&(out(154)==50)
                %data length is 50 in front of every 50 bytes data
                indexJ = 1;
                %remove added charaters
                %when tranfer the mat file, an extra character,length of the string,
                %was added in front of the data for each data transfer.
                for indexI = 1:length(out)
                    if rem(indexI-1, 51)
                        matFile(indexJ) = out(indexI);
                        indexJ = indexJ+1;
                    end
                end
                %We used two commands in order to synchronize sd card information in 2011b. 
                %one command is 'sync_sd_info' and the other is 'get_version'
                %The reply of 'get_version' can trigger the byteavailable
                %event
                matFile = matFile(1:end-31); %Remove the answer from the current version query 
                load('Pcontrol_paths.mat');
                SDfile = fullfile(controller_path, 'SD.mat');
                sdFid = fopen(SDfile, 'w');
                fwrite(sdFid, matFile);
                fclose(sdFid);
                newString = 'The SD.mat is tranferred to PC. Please restart PControl.';
            else %the data are not SD.mat
                newString = char(out)';
            end
            
        elseif byteLen > 0  % 0 < byteLen < 250
             messRevd = fscanf(obj);
             newString = strtrim(messRevd);  %found a hardware reset
             if strcmp(newString, 'Main Controller Works')
                 initialized_PControl_display();
             end
             if strncmp(newString, 'update:', 7);
                update_display_xy(newString);
             end
        else  %byteLen = 0
            return;
        end
end

update_status_display(newString);