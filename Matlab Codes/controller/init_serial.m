function result = init_serial()

global serialPort myPCCfg;


if isfield(myPCCfg, 'portNum')
    portNumber = myPCCfg.portNum;
else
    portNumber = uiserialPort();
end


serialPortName = ['COM', num2str(portNumber)];

try
    %clear existed serial port obj
    serialObj = instrfind('Port',serialPortName);
    if size(serialObj)
        delete(serialObj);
        clear serialObj;
    end

    setSerialPort;
    result = 1;
    
catch ME
    portNumber = uiserialPort();
    serialPortName = ['COM',num2str(portNumber)];

    try
        setSerialPort;
        result = 1;
    catch ME
        disp('Open PControl without serial port connection');
        result = -1;
    end
    
end

    function setSerialPort()
        %!mode com8:115200,n,8,1
        serialPort = serial(serialPortName);
        serialPort.BaudRate = 921600;
        %serialPort.BaudRate = 115200;
        serialPort.DataBits=8;
        serialPort.Parity='none';
        serialPort.StopBits=1;
        serialPort.Terminator='LF';

        serialPort.BytesAvailableFcnMode = 'terminator';
        serialPort.BytesAvailableFcn = @displayOnGui;
        serialPort.ErrorFcn = @displayOnGui;


        %serialPort.InputBufferSize=task*12+1000;
        serialPort.OutputBufferSize=1550;
        serialPort.TimeOut=5;
        serialPort.InputBufferSize = 10000;

        fopen(serialPort);
        disp(serialPort);
    end

    function portNumber = uiserialPort()
        prompt={'Enter the serial port number:'};
        name='Input serial port number';
        numlines=1;
        defaultanswer={'3'};
        answer=inputdlg(prompt,name,numlines,defaultanswer);
        portNumber = str2num(answer{1});
        myPCCfg.portNum = portNumber;
    end
end
