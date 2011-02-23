function Pat = Pattern_Repeater(P, n)
% This function takes in a pattern P (just a matrix) and repeats it in both row and column diurections to 
% form the matrix Pat of size n*size(P). This is especially useful for
% making block patterns from random matrix,
% e.g. R = [0 1 1 0]
% Pat = Pattern_Repeater(R, 2)
% yields:
% [ 0 0 1 1 1 1 0 0;
%   0 0 1 1 1 1 0 0 ]

Pat = zeros(n*size(P));
[r,c] = size(P);

for i = 1:r
    for j = 1:c
        row_ind = (n*(i-1)+1):n*(i);
        col_ind = (n*(j-1)+1):n*(j);
        Pat(row_ind, col_ind) = P(i,j);
    end
end

