function Mout = simple_expansion(Min, exp_col, diameter)
% Min is the matrix in, always shifts out (expansion), and by one frame at
% a time. Also always wrap around to pole of expansion - works best for even patterns 
% diameter is in columns, e,g 8 pans -> 64  

[numR, numC] = size(Min);
Mout = Min;   %to initialize, set the output M equal to the input M


% M1 = Min(:,exp_col+1:exp_col+1 + (diameter/2))                         % 'expanding side'
% M2 = [Min(:,exp_col+ 1 + (diameter/2)+1:end) Min(:,1:exp_col)]         % 'contracting side'
% 
% M_temp = [M2 M1];
% 
% M_temp_o(:,exp_col + 1:end) = M_temp(:,exp_col:end-1);
% M_temp_o(:,exp_col) = M_temp(:,end);
% 
% M_temp_o(:,1:exp_col-2) = M_temp(:,2:exp_col-1);
% M_temp_o(:,exp_col-1) = M_temp(:,1);
% 
% Mout(:,1:exp_col + 1 + (diameter/2))  = M_temp_o(:, numC - exp_col + 1 + (diameter/2) :end);
% Mout(:, exp_col+ 1 + (diameter/2)+1:end) = M_temp_o(:,1:exp_col)]  
 

Mout(:,exp_col + 1:end) = Min(:,exp_col:end-1);
Mout(:,exp_col) = Min(:,end);

Mout(:,1:exp_col-2) = Min(:,2:exp_col-1);
Mout(:,exp_col-1) = Min(:,1);
