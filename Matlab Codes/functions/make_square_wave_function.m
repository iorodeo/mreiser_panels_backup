function func = make_square_wave_function(time, sample_rate, frequency, duty_cycle)
% func = make_square_wave_function(time, sample_rate, frequency, duty_cycle)
% makes a square wave between 0 and 1, for the given time, sample_rate, frequency, 
% and duty cycle. After running function, can then add offset or scale the waveform.
% func = make_square_wave_function(10, 100, 0.2, 0.5)

step_size = 1/sample_rate;
t = 0:step_size:(time - step_size);
num_points = length(t);

period = round((1/frequency)*sample_rate);    % get the period of the square wave in # of samples

high_time = round(duty_cycle*period);
low_time = period - high_time;

%just increment along the length of the vector, alternating high and low time
cur_samp = 1;
while (cur_samp < num_points)
    func(cur_samp:cur_samp + high_time - 1) = 1;
    cur_samp = cur_samp + high_time;
    func(cur_samp:cur_samp + low_time - 1) = 0;
    cur_samp = cur_samp + low_time; 
end

% truncate is it's too long
func = func(1:num_points);
