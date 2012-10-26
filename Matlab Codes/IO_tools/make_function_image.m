function SD = make_function_image(file_list)
% this is the gui-friendly version of the file prepare_flash_image
%% file_list is a structure with fields FileName and PathName

block_size = 512; % all data must be in units of block size
num_functions = length(file_list);
Header_block = zeros(1, block_size);
SD.numfunc = num_functions;

%clean the temp folder
load('Pcontrol_paths.mat');
%dos(['del /Q ' temp_path '\*.fun']); %SS
dos(['del /Q "' temp_path '\*.fun"']); %SS
pos_func_counter = 0;
vel_func_counter = 0;

for j = 1:num_functions
    load([file_list(j).PathName '\' file_list(j).FileName]);
    
    Header_block(1:4) = dec2char(length(func)*2, 4);     %each function datum is stored in two bytes in the SD card
    Header_block(5) = length(file_list(j).FileName);
    Header_block(6: 6 + length(file_list(j).FileName) -1) = file_list(j).FileName;
 
    % set up SD structure with function info
    SD.functionName{j} = file_list(j).FileName;
    SD.functionSize{j} = length(func)*2; 
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
    
    Data_to_write = [Header_block function_Data]; 
    fid = fopen([temp_path '\' funcFileName] , 'w');
    fwrite(fid, Data_to_write(:),'uchar');
    fclose(fid);
    display([num2str(j) ' of ' num2str(num_functions) ' functions written to temporary folder', funcFileName, ', of size ' num2str(size(Data_to_write,2)) ' bytes']);
end

SD.numVelFunc = vel_func_counter;
SD.numPosFunc = pos_func_counter;

