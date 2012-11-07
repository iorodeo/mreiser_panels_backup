% make_6_wide_med_cont_pattern_48.m

pattern.x_num = 96; 	% There are 96 pixel around the display (12x8) 
pattern.y_num = 4; 		% two frames of Y, at 2 different spatial frequencies
pattern.num_panels = 48; 	% This is the number of unique Panel IDs required.
pattern.gs_val = 1; 	% This pattern will use 8 intensity levels
pattern.row_compression = 1;

Pats = zeros(4, 96, pattern.x_num, pattern.y_num);
Pats(:, :, 1, 1) = [ones(4,90) zeros(4,6)]; % one stripe
% two stripes, 90 degs apart
Pats(:, :, 1, 2) = [ones(4,6) zeros(4,6) ones(4,66) zeros(4,6) ones(4,12)]; 
% 3 stripes
Pats(:, :, 1, 3) = repmat([ones(4, 26) zeros(4,6)], 1, 3);
% 8 stripes
Pats(:, :, 1, 4) = repmat([ones(4, 6) zeros(4,6)], 1, 8);

for j = 2:96
    Pats(:,:,j,1) = ShiftMatrix(Pats(:,:,j-1,1),1,'r','y');
    Pats(:,:,j,2) = ShiftMatrix(Pats(:,:,j-1,2),1,'r','y');
    Pats(:,:,j,3) = ShiftMatrix(Pats(:,:,j-1,3),1,'r','y');
    Pats(:,:,j,4) = ShiftMatrix(Pats(:,:,j-1,4),1,'r','y');
end

pattern.Pats = Pats;
pattern.Panel_map = [12 8 4 11 7 3 10 6 2  9 5 1; 24 20 16 23 19 15 22 18 14 21 17 13; 36 32 28 35 31 27 34 30 26 33 29 25; 48 44 40 47 43 39 46 42 38 45 41 37];
% A = 1:48;
% pattern.Panel_map = flipud(reshape(A, 4, 12));
% %     4     8    12    16    20    24    28    32    36    40    44    48
% %     3     7    11    15    19    23    27    31    35    39    43    47
% %     2     6    10    14    18    22    26    30    34    38    42    46
% %     1     5     9    13    17    21    25    29    33    37    41    45
directory_name = 'C:\temp';
pattern.BitMapIndex = process_panel_map(pattern);
pattern.data = make_pattern_vector(pattern);

% directory_name = 'c:\matlabroot\Panels\Patterns';
str = [directory_name '\Pattern_6_wide_stripes_48_Pan']
save(str, 'pattern');
