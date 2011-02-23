% makes a sine wave grating pattern with grayscale and row compression

pattern.x_num = 96; 	% There are 96 pixel around the display (12x8) 
pattern.y_num = 2; 		% two frames of Y, at 2 different spatial frequencies
pattern.num_panels = 48; 	% This is the number of unique Panel IDs required.
pattern.gs_val = 3; 	% This pattern will use 8 intensity levels
pattern.row_compression = 1;

Pats = zeros(4, 96, pattern.x_num, pattern.y_num); 	%initializes the array with zeros
% make grating patterns, periods are 120 and 60 degrees, using all 8 gscale values
Pats(:, :, 1, 1) = repmat(round(3.5*(sin((6*pi/96)*[0:95])+1) ), 4, 1);
Pats(:, :, 1, 2) = repmat(round(3.5*(sin((12*pi/96)*[0:95])+1) ), 4, 1);

for j = 2:96 			%use ShiftMatrixPats to rotate stripe image
    Pats(:,:,j,1) = ShiftMatrix(Pats(:,:,j-1,1),1,'r','y');
    Pats(:,:,j,2) = ShiftMatrix(Pats(:,:,j-1,2),1,'r','y');
end

pattern.Pats = Pats; 		% put data in structure 
A = 1:48;              	% define panel structure vector
pattern.Panel_map = flipud(reshape(A, 4, 12));
pattern.BitMapIndex = process_panel_map(pattern);
pattern.data = make_pattern_vector(pattern);
directory_name = 'c:\matlabroot\Panels\Patterns';
str = [directory_name '\Pattern_grating_row_comp'] 	% name must begin with ‘Pattern_’
save(str, 'pattern');
