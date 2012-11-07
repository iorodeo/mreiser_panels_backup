function PatNums = BinPats(Pats)
% This function converts 0,1 values to ascii characters.

[PatR, PatC, NumPats] = size(Pats);
%NumChars = PatR*NumPats;
numCol = PatC/8

PatNums = [];

for j = 1:NumPats
        for k = 1:numCol
            for i = 1:PatR
            PatNums = [PatNums binvec2dec(Pats(i, 8*k:-1:8*(k-1)+1,j))];
    end
end
end


