% make_stripes_test_patterns.m

% 12 panel pattern
pattern.x_num = 4;
pattern.y_num = 1;
pattern.num_panels = 12;
pattern.gs_val = 1;

InitPat = repmat([zeros(8,2), ones(8,2)], 1,24);
Pats = zeros(8, 96, pattern.x_num, pattern.y_num);

Pats(:,:,1,1) = InitPat;

for j = 2:4
    Pats(:,:,j,1) = ShiftMatrix(Pats(:,:,j-1,1), 1, 'r', 'y'); 
end

pattern.Pats = Pats;
pattern.Panel_map = [1 2 3 4 5 6 7 8 9 10 11 12];
pattern.BitMapIndex = process_panel_map(pattern.Panel_map);
pattern.data = make_pattern_vector(pattern);

directory_name = 'c:\matlabroot\Panels\Patterns';
str = [directory_name '\Pattern_stripes_12Pan']
save(str, 'pattern');

