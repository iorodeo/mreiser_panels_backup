function pat_vector = make_pattern_vector_gs(Pats, BitMapIndex, gs_val)
% converts a Pats file of size (L,M,N,O), where L is the number of rows, 8
% per panel, M is the number of columns, 8 per panel, N is the number of
% frames in the 'x' dimmension, and O is the number of frames in the 'y' dimmension
% to an array of size L/8, M/8, N*O stored as: Pannel, Frame, PatternData 
% here we flatten the 2D pattern array to a 1 D array using the formula 
% Pattern_number = (index_y - 1)*N + index_x;

[PatR, PatC, NumPatsX, NumPatsY] = size(Pats)

NumPats = NumPatsX*NumPatsY;   %total number of panels
numCol = PatC/8;

NumPanels = length(BitMapIndex);   % this count includes virtual panels too, ones with flag = 0
pat_matrix = zeros(NumPanels*NumPats, 8*gs_val);

for index_x = 1:NumPatsX
        for index_y = 1:NumPatsY
            [ index_x index_y ]
            %compute the pattern_number:
            Pattern_number = (index_y - 1)*NumPatsX + index_x;
            for i = 1:NumPanels
                % capture the panel bitmap for frame Pattern_number and panel i
                PanMat = Pats(BitMapIndex(i).row_range, BitMapIndex(i).column_range, index_x, index_y);
                
                for k = 1:8
                    % code below is perfectly general - just treat gs = 1, separately to speed up.
                    if (gs_val == 1)
                        frame_pat(i,k) = vec2dec_fast(PanMat(:,k)',gs_val);                        
                    else
                        out = vec2dec_fast(PanMat(:,k)', gs_val);
                        for num_g = 1:gs_val
                            frame_pat(i,k + (8*(num_g-1))) = out(num_g);
                        end
                    end
                end
            end
                        
            pat_start_index = (Pattern_number - 1)*NumPanels + 1;
            pat_matrix(pat_start_index:pat_start_index + NumPanels - 1, :) = frame_pat;
        end    
end

% rearrange the data so we can read this as columnwise vector data - 
temp_matrix = pat_matrix';
pat_vector = temp_matrix(:);
