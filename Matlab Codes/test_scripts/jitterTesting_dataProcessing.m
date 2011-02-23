sampleRate = 10000;
runTimeforEachRate = 10; %unit second
stopTimeforEachRate = 1; %unit second
oldPath = pwd;
newPath = 'C:\temp';
cd(newPath);

[filename, pathname, filterindex] = uigetfile('*.daq', 'Pick an daq-file');

filename = fullfile(pathname, filename);
[data, time] = daqread(filename);
plot(data);
cd(oldPath);

%find startPints and endPoints
dataIndex = find(data>3);
diffIndex = diff(dataIndex);
endPointIndex = find(diffIndex > ((stopTimeforEachRate) * sampleRate) - 100);
endPoint = dataIndex(endPointIndex);
startPoint = dataIndex(endPointIndex + 1);

%add the first startPoint
startPoint = [dataIndex(1); startPoint];
    

%find the last endPoint
lastStartPoint = startPoint(end);
searchArea = [lastStartPoint+ ((runTimeforEachRate -1) * sampleRate):1:length(data)];
dataInSearchArea = data(searchArea);
diffData = diff(dataInSearchArea);
lastEndPointIndex = find(diffData< -2);
lastEndPointIndex = lastEndPointIndex(end);
lastEndPoint = searchArea(lastEndPointIndex);

endPoint = [endPoint;lastEndPoint];

%process data for each frame rate
if length(startPoint) == length(endPoint)
    for i= 1: length(startPoint)
        validArea = [startPoint(i):1:endPoint(i)];
        diffData = diff(data(validArea));
        upPointsIndex = find(diffData>3);
        downPointsIndex = find(diffData<-1);
        %To do: filter noise signal           
        

        upPoints = validArea(upPointsIndex);
        downPoints = validArea(downPointsIndex);
        
        noisePoints = find(diff(downPoints)<30);
        downPoints = setdiff(downPoints, downPoints(noisePoints));
        
        timeInterval = diff(downPoints)/sampleRate;  %in seconds

        minIFI(i) = min(timeInterval);
        maxIFI(i) = max(timeInterval);
        meanIFI(i) = mean(timeInterval);
        stdIFI(i) = std(timeInterval);
    end
else
    disp('there are errors when finding start points and end points!/n');
end