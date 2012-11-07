function Pats = MakePat(PatSize, PMaker, Pinit)
%MakePat.m
% e.g. for these values
%PatSize = [8 8];
%NumFrames = 10;
%PMaker(1).Command = 'ShiftMatrix(Min, 1, ''r'', ''y'')'
%PMaker(1).Times = 3;

NumSteps = length(PMaker);
SumFrames = 0;
for j = 1:NumSteps
    SumFrames = SumFrames + PMaker(j).Times;
end
SumFrames

% if NumFrames ~= SumFrames
%     error('The NumFrames differs from the number of commanded frames');
% end

Pats = zeros([PatSize, SumFrames]);
Pats(:,:,1) = Pinit;

PatNum = 1;

for j = 1:NumSteps
    NumRepeat = PMaker(j).Times;
    for i = 1:NumRepeat
        M = Pats(:,:,PatNum);           
        PatNum = PatNum + 1;
        Mout = eval(PMaker(j).Command);
        Pats(:,:,PatNum) = Mout;
    end
end

    
