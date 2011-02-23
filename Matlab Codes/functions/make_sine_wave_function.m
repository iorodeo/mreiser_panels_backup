function func = make_sine_wave_function(time, sample_rate, frequency)
% func = make_sine_wave_function(time, sample_rate, frequency)
% makes a sine wave of amplitude 1 for the given time, sample_rate, and
% frequency, to change the phase need to makes small modification.
% example use is : func = make_sine_wave_function(10, 100, 0.2);

step_size = 1/sample_rate;
t = 0:step_size:(time - step_size);
func = sin(2*pi*frequency*t);
