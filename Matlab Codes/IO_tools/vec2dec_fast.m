function out = vec2dec_fast(vec, gs)
% VEC2DEC_FAST Convert binary vector to decimal number fast.
%
%    VEC2DEC_FAST(B, gs) interprets the vector B and returns the
%    equivalent decimal number.  The least significant bit is 
%    represented by the first column.
%
% 
%    Note: The vector cannot exceed 52 values.
%
%    Example:
%       vec2dec_fast([1 1 1 0 1],1) returns 23
%
%jinyang liu 


% Error if vec is not defined.
if isempty(vec)
   error('daq:vec2dec:argcheck', 'B must be defined.  Type ''daqhelp binvec2dec'' for more information.');
end

% Error if vec is not a double.
if (~isa(vec, 'double') | ( any(vec > (2^(gs)-1)) | any(vec < 0) ) )
   error('B must be a gsvec, 0 - 2^(gs)-1');
end

if gs~= 1
    for j = 1:length(vec)
        %binvec(j,:) = dec2bin(vec(j),gs);
        d = vec(j);
        [f,e]=log2(max(d));
        binvec(j,:) = rem(floor(d*pow2(1-max(gs,e):0)),2);
    end
    
    % Convert the binary string to a decimal number.
    
    n=length(vec);
    
    % Convert to numbers
    twos = pow2(n-1:-1:0);
    
    % then just turn each binvec into a dec value
    for j = 1:gs
        out(j) = twos*flipud(binvec(:,j));
    end
    
else
    % Convert the binvec [0 0 1 1] to a binary string '1100';
    h = fliplr(vec);
    
    % Convert the binary string to a decimal number.
    
    [m,n] = size(h);
    
    % Convert to numbers
    twos = pow2(n-1:-1:0);
    
    out = twos*h';

end