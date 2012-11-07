function char_val =signed_16Bit_to_char(B)
% this functions makes two char value (0-255) from a signed
% 16bit valued number in the range of -32767 ~ 32767

if ((any(B > 32767)) || (any(B < -32767)))
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
temp_val = mod(65536 + B, 65536);
for cnt =1 : length(temp_val)
    char_val(2*cnt-1:2*cnt) = dec2char(temp_val(cnt),2);
end
