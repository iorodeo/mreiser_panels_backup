function PControllerUnitTesting

clc;

fprintf(1, 'Please run this testing script after you run PControl and keep PControl GUI on\n');
menuString = sprintf('\nChoose a feature to test:\n\t[1] ADC/DAC testing\n\t[2] Pattern benchmark testing\n\t[3] Pattern Jitter testing\n\t[4] Dump frame testing\n\t[q] Quit\n\n\tSelection: ');

again = 1;
while again
    choice = lower(input(menuString, 's'));
    
    try
        switch choice
            case '1'
                ADCDAC_testing;
            case '2'
                BenchmarkTest;
            case '3'
                JitterTest;
            case '4'
                DumpFrame;
            case 'q'
                return;
            otherwise
                fprintf(2, '\nUnrecognized option ''%s''\n\n', choice);
        end
    catch
        fprintf(2, 'An error occurred while executing a specific test: %s', lasterr);
        return;
    end
    
    %clear
    fprintf(1, '\n');
end

return;

%-----------------------------------------------------
function ADCDAC_testing

try
    fprintf(1, '\n\n\t***************************************\n');
    fprintf(1,     '\t*        ADC/DAC Test                 *\n');
    fprintf(1,     '\t***************************************\n');
    
    prompt = {'Please connect DAC0 to an ADC channel to be tested and connect DAC1 to a scope. You should see a 0 - 4 Volt triangle wave for about 20 seconds. Enter the ADC channel to be tested, from 0 to 3'};
    dlg_title = 'test ADC';
    num_lines= 1; def = {'0'};
    answer  = inputdlg(prompt,dlg_title,num_lines,def);
    
    % If choose cancel, return.
    if isempty(answer)
        return;
    end
    
    % do some error checking
    num_answer = str2double(answer);
    if ( ~isnan(num_answer) && (num_answer == round(num_answer)) && (num_answer >= 0) && (num_answer <= 3) )
        Panel_com('adc_test', [num_answer]);
    else    %otherwise, error and do nothing
        errordlg('ADC channel to test must be a positive integer from 0 to 3 -- no action taken','Bad Input','modal')
    end
    
catch
    
    fprintf(2, '\n\UnitTest-Error: %s\n\n', lasterr);
end

return;

%-----------------------------------------------------
function BenchmarkTest
try
    fprintf(1, '\n\n\t***************************************\n');
    fprintf(1,     '\t*     Pattern benchmark test          *\n');
    fprintf(1,     '\t***************************************\n');
    

    answer = questdlg('please make sure you have load a pattern, such as the Pattern_4x4_blocks_48 in the Pattterns folder, to the SD card using PControl. Are you ready to do the benchmark test?',...
        'Ready to go?', 'Yes', 'No','Yes');
    
    switch answer,
        case 'Yes'
            Panel_com('set_pattern_id', 1);
            Panel_com('bench_pattern');
            pause(5);
        case 'No'
            return;
    end
        
catch
    
    fprintf(2, '\nUnitTest-Error: %s\n\n', lasterr);
end

return;

%-----------------------------------------------------
function JitterTest

try
    fprintf(1, '\n\n\t***************************************\n');
    fprintf(1,     '\t*     Dump frame test                 *\n');
    fprintf(1,     '\t***************************************\n');
    
    
    prompt = {'Please connect int1 to a DAQ AI0 channel. Check and input the device number'};
    dlg_title = 'Connect DAQ card';
    num_lines= 1; def = {'2'};
    answer  = inputdlg(prompt,dlg_title,num_lines,def);
    
    Panel_com('set_pattern_id', 1);
    
    %create analog input channel and logging mode, NI-USB6009
    ai = analoginput('nidaq', ['Dev', answer{1}]);
    
    %AI0 and AI4 are the postive and negative inputs of differential analog input channel 0
    chans = addchannel(ai, 0);
     set(ai, 'SampleRate', 10000);
     set(ai,'LoggingMode','Disk&Memory');
     set(ai,'LogFileName','c:\temp\jitter1.daq');
     set(ai,'SamplesPerTrigger', inf);
    
    %start(ai);
    pause(1);
    %for gs = 1 (rc=0), benchmark rate is about 200 frame per second
    for i = 10:10:100
        Panel_com('send_gain_bias',[i,0,0,0]);
        Panel_com('start');
        pause(10);
        Panel_com('stop');
        pause(1);
    end
    
    stop(ai);
    
    %Data processing the logged file
    sampleRate = 10000;
    runTimeforEachRate = 10; %unit second
    stopTimeforEachRate = 1; %unit second

    
    filename = 'c:\temp\jitter1.daq';
    [data, time] = daqread(filename);
    plot(data);
    
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
    
catch
    
    fprintf(2, '\nUnitTest-Error: %s\n\n', lasterr);
end

return;

%----------------------------------------------------------

function DumpFrame

global frame_num frame_length pattern counter

try
    fprintf(1, '\n\n\t***************************************\n');
    fprintf(1,     '\t*     Dump frame testing              *\n');
    fprintf(1,     '\t***************************************\n');
    
% enter PC dumping mode    
Panel_com('pc_dumping_mode');
pause(2);
Panel_com('ctr_reset');
fprintf(1, '\nenter PC dumping mode!\n');
pause(10);

fileName = mfilename('fullpath');
[pathstr, ~] = fileparts(fileName); 
newPath = [pathstr(1:end-12), 'Patterns']; 
load([newPath, '\Pattern_4x4_blocks_48.mat']);
pattern.row_compression = 0;

if pattern.row_compression == 1
    frame_length = pattern.gs_val * pattern.num_panels;
else
    frame_length = pattern.gs_val * pattern.num_panels * 8;
end
    
t= timer;
counter = 1;
set(t,'ExecutionMode','fixedrate');
frame_num = 0;

t.TimerFcn = @timefun;
set(t,'tasksToExecute', 1000);

i=50;
set(t,'Period',1/i);
ts=tic;
start(t);
pause(1000/i +5);
stop(t);
telapsed=toc(ts)

% return controller mode
Panel_com('controller_mode');
pause(2);
Panel_com('ctr_reset');
fprintf(1, '\nreturn to controller mode!\n');
pause(8);

catch
    
    fprintf(2, '\nUnitTest-Error: %s\n\n', lasterr);
end

return;

function timefun(obj, event)
    global frame_num frame_length pattern counter
    tstart = tic;
    data = transpose(pattern.data(frame_num*frame_length+1:(frame_num + 1)*frame_length));
    Panel_com('dump_frame', [frame_length, frame_num/pattern.x_num * 2047, 0, pattern.num_panels,  pattern.gs_val, pattern.row_compression, data]);
    frame_num = mod(frame_num + 1, pattern.x_num);
    counter = counter + 1;
    etime{counter}=toc(tstart);
    

