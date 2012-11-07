% Closed_Loop_random_patterns

CL_Pat = 2;      % uses pattern 2
num_patterns = 4;

time_CL = 20;    %45;
num_repeats = 4;

gain_x = -20;
bias_x = 0;

AO = analogoutput('mcc',0);
ch = addchannel(AO, [0 1]);

Panel_com('set_mode', [ 1 0 ]);                        
Panel_com('send_gain_bias', [gain_x, bias_x, 0, 0]);             
    
Panel_com('set_pattern_id', CL_Pat);            
Panel_com('stop');

putsample(AO, [0 0]);
fprintf('first trial: plain stripe fixation...');
Panel_com('set_position', [48 1]);  
panel_com('start');
pause(time_CL);
Panel_com('stop');
fprintf('pause \n');

for i = 1:num_repeats      
    rand_ind_CL = randperm(num_patterns)
    for j = 1:num_patterns
        Pattern_id = rand_ind_CL(j);
        
        Panel_com('set_position', [48 Pattern_id]); 
        Panel_com('set_mode', [ 1 0 ]);                        
        Panel_com('send_gain_bias', [gain_x, bias_x, 0, 0]);   
               
        fprintf('trial %d, run %d, pattern_id %d \n',i,j, Pattern_id);
       
        putsample(AO, [Pattern_id 0]);
        panel_com('start');
        pause(time_CL);
        Panel_com('stop');
       
    end       
end   