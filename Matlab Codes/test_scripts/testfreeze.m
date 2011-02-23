for k=1:1000
    for i =1:14
            for j = 0:7
                %Panel_com('all_off')
                %pause(2);
                fprintf('Start a new trial: i=%d, j=%d, tolNum = %d\n', i,j,(k-1)*14*8+(i-1)*8+j)
                Panel_com('set_posfunc_id', [1 j])
                Panel_com('set_posfunc_id', [2 0])
                Panel_com('set_mode',[4 0]);
                Panel_com('send_gain_bias',[100 0 0 0]);
                Panel_com('set_pattern_id', i);
                Panel_com('set_position', [1 1]);
                Panel_com('start')
                pause(2);
                Panel_com('stop')
                

            end
    end

end

    
    

             
