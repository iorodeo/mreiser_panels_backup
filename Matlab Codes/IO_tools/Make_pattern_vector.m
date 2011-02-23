function pat_vector = make_pattern_vector(pattern)
% relevant fields of pattern are - Pats, BitMapIndex, gs_val
% converts a Pats file of size (L,M,N,O), where L is the number of rows, 8
% per panel, M is the number of columns, 8 per panel, N is the number of
% frames in the 'x' dimmension, and O is the number of frames in the 'y' dimmension
% to an array of size L/8, M/8, N*O stored as: Pannel, Frame, PatternData 
% here we flatten the 2D pattern array to a 1 D array using the formula 
% Pattern_number = (index_y - 1)*N + index_x;

Pats = pattern.Pats; 
BitMapIndex = pattern.BitMapIndex; 
gs_val = pattern.gs_val;

% first we do some error checking
if (~( (gs_val == 1) | (gs_val == 2) | (gs_val == 3) | (gs_val == 4) ))
    error('gs_val must be 1, 2, 3, or 4');
end

if ( (gs_val == 1) & (~all( (Pats(:) == 0) | (Pats(:) == 1) ) ) )
        error('For gs 1, Pats can contain only 0 or 1');
end

if ( (gs_val == 2)& (~all((Pats(:) == 0) | (Pats(:) == 1) | (Pats(:) == 2) | (Pats(:) == 3) ) ) )
        error('For gs 2, Pats can contain only 0, 1, 2, or 3');
end

if ( (gs_val == 3) & (~all((Pats(:) == 0) | (Pats(:) == 1) | (Pats(:) == 2) ...
        | (Pats(:) == 3) | (Pats(:) == 4) | (Pats(:) == 5) | (Pats(:) == 6) | (Pats(:) == 7) ) ) )
        error('For gs 3, Pats can contain only 0, 1, 2, 3, 4, 5, 6, or 7');
end

if ( (gs_val == 4) & (~all((Pats(:) == 0) | (Pats(:) == 1) | (Pats(:) == 2) ...
        | (Pats(:) == 3) | (Pats(:) == 4) | (Pats(:) == 5) | (Pats(:) == 6) | (Pats(:) == 7)...
        | (Pats(:) == 8) | (Pats(:) == 9) | (Pats(:) == 10) | (Pats(:) == 11) | (Pats(:) == 12)...
        | (Pats(:) == 13) | (Pats(:) == 14) | (Pats(:) == 15) ) ) )
        error('For gs 4, Pats can contain only 0-15');
end

% determine if row_compression is on  
row_compression = 0;
if isfield(pattern, 'row_compression') % for backward compatibility
    if (pattern.row_compression)
        row_compression = 1;
    end
end

[PatR, PatC, NumPatsX, NumPatsY] = size(Pats);

NumPats = NumPatsX*NumPatsY;   %total number of panels
numCol = PatC/8;

NumPanels = length(BitMapIndex);   % this count includes virtual panels too, ones with flag = 0
if (row_compression)
    pat_matrix = zeros(NumPanels*NumPats, 1*gs_val);
else    
    pat_matrix = zeros(NumPanels*NumPats, 8*gs_val);
end    

for index_x = 1:NumPatsX
        for index_y = 1:NumPatsY
            [ index_x index_y ]
            %compute the pattern_number:
            Pattern_number = (index_y - 1)*NumPatsX + index_x;
            for i = 1:NumPanels
                % capture the panel bitmap for frame Pattern_number and panel i
                PanMat = Pats(BitMapIndex(i).row_range, BitMapIndex(i).column_range, index_x, index_y);
                if (row_compression)
                    if (gs_val == 1)
                        frame_pat(i) = vec2dec_fast(PanMat, gs_val);  
                    else % only support the gs_val <= 4 case
                        if (gs_val > 4)
                            error('gs_val = 1-4 cases are supported!');
                        end
                        out = vec2dec_fast(PanMat, gs_val);
                        for num_g = 1:gs_val
                            frame_pat(i,num_g) = out(num_g);
                        end % for                        
                    end  % if/else gs_val                    
                else    
                    for k = 1:8
                        % code below is perfectly general - just treat gs = 1, separately to speed up.
                        if (gs_val == 1)
                            %frame_pat(i,k) = (binvec2dec(PanMat(:,k)'));                        
                            frame_pat(i,k) = vec2dec_fast(PanMat(:,k)',gs_val);                        
                        else
                            if (gs_val > 4)
                                error('gs_val = 1-4 cases are supported!');
                            end    
                            out = vec2dec_fast(PanMat(:,k)', gs_val);
                            for num_g = 1:gs_val
                                frame_pat(i,k + (8*(num_g-1))) = out(num_g);
                            end % for
                        end %if/else gs_val
                    end % for
                end % if/else row_compression
            end                        
            pat_start_index = (Pattern_number - 1)*NumPanels + 1;
            pat_matrix(pat_start_index:pat_start_index + NumPanels - 1, :) = frame_pat;
        end    
end

% rearrange the data so we can read this as columnwise vector data - 
temp_matrix = pat_matrix';
pat_vector = temp_matrix(:);
