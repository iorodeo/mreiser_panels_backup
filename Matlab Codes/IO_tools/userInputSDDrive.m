function SDdrive = userInputSDDrive()

prompt={'Enter the drive letter for the SD card:'};
name='Input SD card drive letter';
numlines=1;
defaultanswer={'E'};
answer=inputdlg(prompt,name,numlines,defaultanswer);
SDdrive = answer{1};

if ~ischar(answer{1})
    SDdrive = -1;
end