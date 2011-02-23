function SD_image = open_SD_image()

panel_control_paths;
fid = fopen([temp_path '\SD.img'] , 'r');
SD_image = fread(fid, inf, 'uchar');
fclose('all');
