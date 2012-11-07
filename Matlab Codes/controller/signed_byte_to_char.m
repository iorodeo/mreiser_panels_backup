function char_val = signed_byte_to_char(B)
% this functions makes a char value (0-255) from a signed
% byte valued number in the range of -127 - 127

if ((any(B > 127)) || (any(B < -127)))
    error('this number is out of range - need a function to handle multi-byte values' );
end

% if (B >= 0)
%     char_val = B;
% else
% %    Bin_array = dec2binvec(-1*B,8);
% %    char_val = binvec2dec(~Bin_array) + 1;
% %    above is correct, but there is a simpler way to do this:
%     char_val = 256 + B;  
% end

% this does both pos and neg in one line
char_val = mod(256 + B, 256);
