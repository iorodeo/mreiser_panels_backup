function SD = make_function_image(file_list)
% this is the gui-friendly version of the file prepare_flash_image
%% file_list is a structure with fields FileName and PathName

block_size = 512; % all data must be in units of block size
num_functions = length(file_list);
Header_block = zeros(1, block_size);
SD.numfunc = num_functions;
FUNCTION_LENGTH = 100;  % this is the buffer read size in panel controller
OFFSET = 0;
%clean the temp folder
load('Pcontrol_paths.mat');
%dos(['del /Q ' temp_path '\*.fun']); %SS
dos(['del /Q "' temp_path '\*.fun"']); %SS
pos_func_counter = 0;

vel_func_counter = 0;

for j = 1:num_functions
    load([file_list(j).PathName file_list(j).FileName]);
    
    Header_block(1:4) = dec2char(length(func)*2, 4);     %each function datum is stored in two bytes in the SD card
    Header_block(5) = length(file_list(j).FileName);
    Header_block(6: 6 + length(file_list(j).FileName) -1) = file_list(j).FileName;
 
    % set up SD structure with function info
    SD.functionName{j} = file_list(j).FileName;
    SD.functionSize{j} = length(func)*2; 
%     num_blocks_needed(j) = ceil(SD.functionSize{j}/FUNCTION_LENGTH);
%     last_load_data(j) = mod(SD.functionSize{j},FUNCTION_LENGTH);
%     padded_function_Data = zeros(1, num_blocks_needed(j)*block_size);
    
    %each function datum is stored in two bytes in the SD card
    if strncmp(file_list(j).FileName, 'function', 8)
        vel_func_counter = vel_func_counter + 1;
        switch length(num2str(vel_func_counter))
            case 1
                funcFileName = ['vel000' num2str(vel_func_counter) '.fun'];
            case 2
                funcFileName = ['vel00', num2str(vel_func_counter) '.fun'];
            case 3
                funcFileName = ['vel0', num2str(vel_func_counter) '.fun'];
            case 4
                funcFileName = ['vel', num2str(vel_func_counter) '.fun'];
            otherwise
                disp('The number of function you choose exceeds the maximum.');
                break;
        end
        SD.velFunctionName{vel_func_counter} = file_list(j).FileName;
        function_Data = signed_16Bit_to_char(round(20.*func));   % 20 = 1V
            
    elseif strncmp(file_list(j).FileName, 'position', 8)
        pos_func_counter = pos_func_counter + 1;
        switch length(num2str(pos_func_counter))
            case 1
                funcFileName = ['pos000' num2str(pos_func_counter) '.fun'];
            case 2
                funcFileName = ['pos00', num2str(pos_func_counter) '.fun'];
            case 3
                funcFileName = ['pos0', num2str(pos_func_counter) '.fun'];
            case 4
                funcFileName = ['pos', num2str(pos_func_counter) '.fun'];
            otherwise
                disp('The number of function you choose exceeds the maximum.');
                break;
        end
        SD.posFunctionName{pos_func_counter} = file_list(j).FileName;
        function_Data = signed_16Bit_to_char(func);     
    
    else
        disp('The function name %s is incorrect\n',file_list(j).FileName);
    end
    %now we add 100 byte OFFSET to speed up the sd card reading spead
    % now write all of the frame info
%     if last_load_data(j) == 0
%         for i = 1:num_blocks_needed(j)
%             sd_start_address = (i - 1)*block_size + OFFSET + 1;
%             sd_end_address = sd_start_address + FUNCTION_LENGTH - 1;
%             % always forced to start frame at a block boundary
%             func_start_address = (i - 1)* FUNCTION_LENGTH + 1;
%             func_end_address = func_start_address + FUNCTION_LENGTH - 1;
%             padded_function_Data(sd_start_address:sd_end_address) = function_Data(func_start_address:func_end_address);
%         end
%         
%     else
%         for i = 1:num_blocks_needed(j)-1
%             sd_start_address = (i - 1)*block_size + OFFSET + 1;
%             sd_end_address = sd_start_address + FUNCTION_LENGTH - 1;
%             % always forced to start frame at a block boundary
%             func_start_address = (i - 1)* FUNCTION_LENGTH + 1;
%             func_end_address = func_start_address + FUNCTION_LENGTH - 1;
%             padded_function_Data(sd_start_address:sd_end_address) = function_Data(func_start_address:func_end_address);
%         end
%         
%         sd_start_address = (num_blocks_needed(j) - 1)*block_size + OFFSET + 1;
%         sd_end_address = sd_start_address + last_load_data(j) - 1;
%         % always forced to start frame at a block boundary
%         func_start_address = (num_blocks_needed(j) - 1)* FUNCTION_LENGTH + 1;
%         func_end_address = func_start_address + last_load_data(j) - 1;
%         padded_function_Data(sd_start_address:sd_end_address) = function_Data(func_start_address:func_end_address);
%     end
        
    Data_to_write = [Header_block function_Data]; 
    fid = fopen([temp_path '\' funcFileName] , 'w');
    fwrite(fid, Data_to_write(:),'uchar');
    fclose(fid);
    display([num2str(j) ' of ' num2str(num_functions) ' functions written to temporary folder', funcFileName, ', of size ' num2str(size(Data_to_write,2)) ' bytes']);
end

SD.numVelFunc = vel_func_counter;
SD.numPosFunc = pos_func_counter;

