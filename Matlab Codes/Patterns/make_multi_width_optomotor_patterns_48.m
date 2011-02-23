% make_6_wide_med_cont_pattern_48.m

pattern.x_num = 96; 	% There are 96 pixel around the display (12x8) 
pattern.y_num = 4; 		% two frames of Y, at 2 different spatial frequencies
pattern.num_panels = 48; 	% This is the number of unique Panel IDs required.
pattern.gs_val = 1; 	% This pattern will use 8 intensity levels
pattern.row_compression = 1;

Pats = zeros(4, 96, pattern.x_num, pattern.y_num);
% 6 pixel period; spat. freq. 22.5 degs/cycle
Pats(:, :, 1, 1) = repmat([ones(4,3) zeros(4,3)], 1, 16); 
% 8 pixel period; spat. freq. 30 degs/cycle
Pats(:, :, 1, 2) = repmat([ones(4,4) zeros(4,4)], 1, 12); 
% 12 pixel period; spat. freq. 45 degs/cycle
Pats(:, :, 1, 3) = repmat([ones(4,6) zeros(4,6)], 1, 8); 
% 16 pixel period; spat. freq. 60 degs/cycle
Pats(:, :, 1, 4) = repmat([ones(4,8) zeros(4,8)], 1, 6); 

for j = 2:96
    Pats(:,:,j,1) = ShiftMatrix(Pats(:,:,j-1,1),1,'r','y');
    Pats(:,:,j,2) = ShiftMatrix(Pats(:,:,j-1,2),1,'r','y');
    Pats(:,:,j,3) = ShiftMatrix(Pats(:,:,j-1,3),1,'r','y');
    Pats(:,:,j,4) = ShiftMatrix(Pats(:,:,j-1,4),1,'r','y');
end

pattern.Pats = Pats;

pattern.Panel_map = [12 8 4 11 7 3 10 6 2  9 5 1; 24 20 16 23 19 15 22 18 14 21 17 13; 36 32 28 35 31 27 34 30 26 33 29 25; 48 44 40 47 43 39 46 42 38 45 41 37];
pattern.BitMapIndex = process_panel_map(pattern);
pattern.data = make_pattern_vector(pattern);

%directory_name = 'c:\matlabroot\Panels\Patterns';
directory_name = 'C:\Documents and Settings\liuj\My Documents\MATLAB\Xmega_Panel_controller\Patterns';
str = [directory_name '\Pattern_multi_width_optomotor_patterns_48_Pan']
save(str, 'pattern');
