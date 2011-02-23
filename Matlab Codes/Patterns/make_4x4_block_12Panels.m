%Make_4x4_blocks_12Panels

% 12 panel pattern
pattern.x_num = 8;
pattern.y_num = 8;
pattern.num_panels = 12;
pattern.gs_val = 1;

% 12 panels in a circle -> 96 columns, 1 panel high -> 8 rows
InitPat = [repmat([zeros(4,4), ones(4,4)], 1,12);repmat([ones(4,4), zeros(4,4)], 1,12)];

Pats = zeros(8, 96, pattern.x_num, pattern.y_num);

Pats(:,:,1,1) = InitPat;

for j = 2:8
    Pats(:,:,1,j) = ShiftMatrix(Pats(:,:,1,j-1), 1, 'r', 'y'); 
end

for j = 1:8
    for i = 2:8
        Pats(:,:,i,j) = ShiftMatrix(Pats(:,:,i-1,j), 1, 'd', 'y'); 
    end
end

pattern.Pats = Pats;
%A = 1:12;
%pattern.Panel_map = flipud(reshape(A, 1, 12));
pattern.Panel_map = [12 8 4 11 7 3 10 6 2 9 5 1];
pattern.BitMapIndex = process_panel_map(pattern);
pattern.data = make_pattern_vector(pattern);

directory_name = 'C:\Documents and Settings\liuj\My Documents\MATLAB\Xmega_Panel_controller\Patterns';
str = [directory_name '\Pattern_4x4_blocks_12']
save(str, 'pattern');