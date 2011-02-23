function m_data = sign_mod(x,y)
% m_data = sign_mod(x,y)
% this function is specifically written for creating offset functions for
% pattern data that wraps frequently, say every 8 frames. If data is
% positive, then mod is as usual, if data is negative, then data is modded
% data - actual number

num_pts = length(x);
for j = 1:num_pts
    if x(j) >= 0
        m_data(j) = mod(x(j), y);
    else
        m_data(j) = - mod(-x(j), y);
    end
end
