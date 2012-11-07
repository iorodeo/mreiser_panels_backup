function BitMap = makeBitMap(numRows, numCols, init)

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
        plot([0.5 numCols + 0.5], [j j],'k');
    end

    %make vertical lines
    for j = 1.5:numCols
        plot([j j], [0.5 numRows + 0.5],'k');
    end

    [X,Y,button] = ginput(1);

    if (button == 1)    % if left click
        RowAddress = ceil(Y - 0.5);
        ColAddress = ceil(X - 0.5);
        BitMap(RowAddress, ColAddress) =  ~BitMap(RowAddress, ColAddress);  %toggle pixel
    else running = 0;        
    end
    
end

    
    
    
