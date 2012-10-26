% make_gs_stripes_pattern_12.m

InitPat = [0*ones(8,12) 1*ones(8,12) 2*ones(8,12) 3*ones(8,12)...
    4*ones(8,12) 5*ones(8,12) 6*ones(8,12) 7*ones(8,12) ];

pattern.x_num = 96;
pattern.y_num = 1;
pattern.num_panels = 12;
pattern.gs_val = 3;

Pats = zeros(8, 96, pattern.x_num, pattern.y_num);
Pats(:,:,1,1) = InitPat;

for j = 2:96
    Pats(:,:,j,1) = ShiftMatrix(Pats(:,:,j-1,1), 1, 'r', 'y'); 
end

pattern.Pats = Pats;
pattern.Panel_map = [12 8 4 11 7 3 10 6 2 9 5 1];
pattern.BitMapIndex = process_panel_map(pattern);
pattern.data = make_pattern_vector(pattern);

directory_name = 'C:\temp';
str = [directory_name '\Pattern_gs_stripes_12Pan']
save(str, 'pattern');
