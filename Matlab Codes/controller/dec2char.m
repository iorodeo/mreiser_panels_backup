function charArray = dec2char(num, num_chars)
% this functions makes an array of char values (0-255) from a decimal number
% this is listed in MSB first order.
% untested for negative numbers, probably wrong!
% to decode, for e.g. a 3 char array:
% ans = charArray(1)*2^16 + charArray(2)*2^8 + charArray(3)*2^0

charArray = zeros(1,num_chars);
if (num > 2^(8*num_chars))
    error('not enough characters for a number of this size' );
end

if (num < 0 )
    error('this function does not handle negative numbers correctly' );
end

num_rem = num;

for j = num_chars:-1:1
    temp = floor((num_rem)/(2^(8*(j-1))));
    num_rem = num_rem - temp*(2^(8*(j-1)));
    charArray(j) = temp;
end





    