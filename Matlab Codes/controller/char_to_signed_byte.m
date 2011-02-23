function B = char_to_signed_byte(char_val)
% this functions makes a char value (0-255) from a signed
% byte valued number in the range of -127 - 127

% this does both pos and neg in one line
%char_val = mod(256 + B, 256);

if (char_val > 127)
    B = char_val - 256;
else
    B = char_val;
end    