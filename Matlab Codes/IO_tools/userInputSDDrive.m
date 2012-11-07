function SDdrive = userInputSDDrive(defaultanswer)

prompt={'Enter the drive letter for the SD card:'};
name='Input SD card drive letter';
numlines=1;

answer=inputdlg(prompt,name,numlines,defaultanswer);

if ~isempty(answer) && ischar(answer{1})
    SDdrive = answer{1};
else
    SDdrive = -1;
end