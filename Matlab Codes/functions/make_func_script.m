% make_func_script.m
% make and save some function files

func_path = 'C:\Documents and Settings\liuj\My Documents\MATLAB\Xmega_Panel_controller\functions\';

func = 2*(make_square_wave_function(20, 50, 0.1, 0.5) - 0.5);
save([ func_path 'function_square_amp1_0.1hz_symmetric.mat'], 'func');

func = 1*(make_square_wave_function(20, 50, 0.05, 0.5) - 0.5);
save([ func_path 'function_square_0.05hz_symmetric.mat'], 'func');

func = 3*(make_square_wave_function(20, 50, 0.1, 0.5) - 0.5);
save( [ func_path 'function_1p5_square_amp1_0.1hz_symmetric.mat'], 'func');

func = 4*(make_square_wave_function(20, 50, 0.1, 0.5) - 0.5);
save( [ func_path 'function_square_amp2_0.1hz_symmetric.mat'], 'func');

func = 6*(make_square_wave_function(20, 50, 0.1, 0.5) - 0.5);
save( [ func_path 'function_square_amp3_0.1hz_symmetric.mat'], 'func');

func = 6*(make_square_wave_function(20, 50, 0.05, 0.5) - 0.5);
save( [ func_path 'function_square_amp3_0.05hz_symmetric.mat'], 'func');

func = 10*(make_square_wave_function(20, 50, 0.1, 0.5) - 0.5);
save( [ func_path 'function_square_amp5_0.1hz_symmetric.mat'], 'func');

func = 10*(make_square_wave_function(20, 50, 0.05, 0.5) - 0.5);
save( [ func_path 'function_square_amp5_0.05hz_symmetric.mat'], 'func');

func = make_sine_wave_function(20, 50, 1);
save( [ func_path 'function_sine_1hz.mat'], 'func');

func = make_sine_wave_function(20, 50, 0.2);
save( [ func_path 'function_sine_0.2hz.mat'], 'func');

func = make_sine_wave_function(20, 50, 0.05);
save( [ func_path 'function_sine_0.05hz.mat'], 'func');

func = make_square_wave_function(20, 50, 2, 0.5);
save( [ func_path 'function_square_2hz_50per.mat'], 'func');

func = make_square_wave_function(20, 50, 0.2, 0.5);
save( [ func_path 'function_square_0.2hz_50per.mat'], 'func');

func = make_square_wave_function(20, 50, 0.05, 0.15);
save( [ func_path 'function_square_0.05hz_15per.mat'], 'func');

func = make_square_wave_function(20, 50, 0.05, 0.1);
save( [ func_path 'function_square_0.05hz_10per.mat'], 'func');

func = make_square_wave_function(20, 50, 0.05, 0.01);
save( [ func_path 'function_square_0.05hz_01per.mat'], 'func');

func = make_square_wave_function(20, 50, 0.1, 0.02);
save( [ func_path 'function_square_0.1hz_02per.mat'], 'func');

func = make_square_wave_function(20, 50, 0.05, 0.05);
save( [ func_path 'function_square_0.05hz_05per.mat'], 'func');

func = zeros(1,1000);
save( [ func_path 'function_DC_zero'], 'func');

func = ones(1,1000);
save( [ func_path 'function_DC_1V'],  'func');

func = (1/20)*ones(1,1000);
save( [ func_path 'function_DC_0p1V'], 'func');

func = make_sine_wave_function(30, 50, 0.5);
save( [ func_path 'function_sine_0.5hz_varlen.mat'], 'func');

func = 3*(make_square_wave_function(40, 50, 0.1, 0.5) - 0.5);
save( [ func_path 'function_1p5_square_amp1_0.1hz_symmetric_var.mat'], 'func');







