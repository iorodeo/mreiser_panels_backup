%make_false_contours.m

%CA
pattern.x_num = 96;
pattern.y_num = 7; 
pattern.num_panels = 48;
pattern.gs_val = 3;

Pats = zeros(32, 96, pattern.x_num, pattern.y_num);

for j =1:7
  Pats(:,:,1,16) = [j*ones(32,44) zeros(32,8) j*ones(32,44)] ;

 for i = 8
  Pats(:,:,1,j) = [j*ones((32-i)/2,96); j*ones(i,44) zeros(i,8) j*ones(i,44); j*ones((32-i)/2,96)] ;

 end
end

% 
for j = 2:96
    for k = 1:pattern.y_num
    Pats(:,:,j,k) = ShiftMatrix(Pats(:,:,j-1,k), 1, 'r', 'y'); 
    end
end

pattern.Pats = Pats;
pattern.Panel_map = [12 8 4 11 7 3 10 6 2  9 5 1; 24 20 16 23 19 15 22 18 14 21 17 13; 36 32 28 35 31 27 34 30 26 33 29 25; 48 44 40 47 43 39 46 42 38 45 41 37];
pattern.BitMapIndex = process_panel_map(pattern);
pattern.data = make_pattern_vector(pattern);
directory_name = 'C:\';
str = [directory_name 'Documents and Settings\liuj\My Documents\MATLAB\Xmega_Panel_controller\Patterns\testpattern.mat'] 	% name must begin with ‘Pattern_’
save(str, 'pattern');
