function BitMap = make_panel_bit_map(n_Panels_inRow, n_Panels_inColumn, init)
%same as other MakeBitMap, but this is defined on a panel basis (mult of
%8), and shows panel boundaries.

numRows = n_Panels_inRow*8;
numCols = n_Panels_inColumn*8;

if (nargin == 3)
    BitMap = init;
    [numRows, numCols] = size(init);
else
    BitMap = zeros(numRows, numCols);
end

iptsetpref('ImshowTrueSize', 'manual')

running = 1;

while(running)
    clf         %clear figure
    imshow(~BitMap);    % show cuurent map, ~ because in matlab 0 shows black
    hold on
    axis tight

    %make horizontal lines
    for j = 1.5:numRows
        if (mod(j,8) == 0.5) plot([0.5 numCols + 0.5], [j j],'r', 'LineWidth',2);
        else plot([0.5 numCols + 0.5], [j j],'k');
        end
    end

    %make vertical lines
    for j = 1.5:numCols
        if (mod(j,8) == 0.5) plot([j j], [0.5 numRows + 0.5],'r', 'LineWidth',2);
        else plot([j j], [0.5 numRows + 0.5],'k');
        end
    end

    [X,Y,button] = ginput(1);

    if (button == 1)    % if left click
        RowAddress = ceil(Y - 0.5);
        ColAddress = ceil(X - 0.5);
        if (ColAddress > numCols)
            BitMap(RowAddress, :) =  ~BitMap(RowAddress, :);  %toggle pixel
        elseif (RowAddress > numRows)
            BitMap(:, ColAddress) =  ~BitMap(:, ColAddress);  %toggle pixel
        else
            BitMap(RowAddress, ColAddress) =  ~BitMap(RowAddress, ColAddress);  %toggle pixel
        end
    else running = 0;        
    end
    
end

    
    
    
