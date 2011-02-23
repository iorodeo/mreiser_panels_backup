function [BitMapIndex] = process_panel_map(pattern)
% takes an argument like [1 2 3 4; 5 6 7 8] to define a tilling by 8 panels
% of a 2x4 grid of panels. 
% if there is a zero term, as in [1 2 3 0; 5 6 7 8], then this signifies a
% dummy panel for which no data will be sent out & patterns won't be
% computed.
% Returns a structure BitMapIndex of length = number of defined panel_IDs
% has 3 fields: Panel_ID, row_range, and column_range.

Panel_map = pattern.Panel_map;
[ n_Panels_inRow, n_Panels_inColumn ] = size(Panel_map);
% bitmap pattern must be size [8*n_Panels_inRow, 8*n_Panels_inColumn];

% determine if row_compression is on  
row_compression = 0;
if isfield(pattern, 'row_compression') % for backward compatibility
    if (pattern.row_compression)
        row_compression = 1;
    end
end

Sorted_indices = sort(Panel_map(:));
GT_zero = find(Sorted_indices);
min_pan_index = GT_zero(1);
Non_zero_indices = Sorted_indices(min_pan_index:end);
if (length(unique(Non_zero_indices)) ~= length(Non_zero_indices))   % this means there are repeated panel IDs
    error([' There are multiple panels defined with the same panel ID.' char(13) ...
        ' The logic this program uses to parse patterns to the panels is not appropriate for this case!']);
end

% only search over the list of sorted indices that are non_zero
for i = 1:length(Non_zero_indices)
    Pan_ID = Non_zero_indices(i);
    [I,J] = find(Panel_map == Pan_ID);
    if (isempty(I)) 
        error('something funny here, this index should be in the Panel_map');
    else
        BitMapIndex(i).Panel_ID = Pan_ID;
        if (row_compression)
            BitMapIndex(i).row_range = ((I-1)+1):I;
        else
            BitMapIndex(i).row_range = ((I-1)*8+1):I*8;
        end
        BitMapIndex(i).column_range = ((J-1)*8+1):J*8;
    end
end
