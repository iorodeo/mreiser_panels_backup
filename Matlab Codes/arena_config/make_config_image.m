function SD = make_config_image(file_list)
% this is the gui-friendly version of the file prepare_flash_image
%% file_list is a structure with fields FileName and PathName

num_configs = length(file_list);
SD.num_configs = num_configs;

%clean the temp folder
load('Pcontrol_paths.mat');
dos(['del /Q ' temp_path '\*.cfg']); 

for j = 1:num_configs
    load([file_list(j).PathName file_list(j).FileName]);

    % set up SD structure with pattern info
    SD.cfgNames{j} = file_list(j).FileName;

    
     Data_to_write = [cfgData(:,2);cfgData(:,4);cfgData(:,6); cfgData(:,8); cfgData(:,10); cfgData(:,12); cfgData(:,14); cfgData(:,16)]';
    
    switch length(num2str(j))
        case 1
            cfgFileName = ['cfg000' num2str(j) '.cfg'];
        case 2
            cfgFileName = ['cfg00', num2str(j) '.cfg'];
        case 3
            cfgFileName = ['cfg0', num2str(j) '.cfg'];
        case 4
            cfgFileName = ['cfg', num2str(j) '.cfg'];
        otherwise
            disp('The pattern number is too big.');
    end
    
    fid = fopen([temp_path '\' cfgFileName] , 'w');
    fwrite(fid, Data_to_write(:),'uchar');
    fclose(fid);
    display([num2str(j) ' of ' num2str(num_configs) ' arena configuraiton written to temporary file ' temp_path,  '\', cfgFileName ', of size '...
        num2str(size(Data_to_write, 1)) 'x' num2str(size(Data_to_write, 2)) ' bytes']);
end

