function SD_image = open_SD_image()

load('Pcontrol_paths.mat');
fid = fopen([temp_path '\SD.img'] , 'r');
SD_image = fread(fid, inf, 'uchar');
fclose('all');
