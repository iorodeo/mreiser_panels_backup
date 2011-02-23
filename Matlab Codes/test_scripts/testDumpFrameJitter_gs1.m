t= timer;
set(t,'ExecutionMode','fixedrate');
%gs = 3;
set(t, 'timerFcn', @(x,y)Panel_com('dump_frame', [384, 48, 1, 0, round(rand(1,384)*255)]));
set(t,'tasksToExecute', 1000);

for i = 10:10:100
    i
    set(t,'Period',1/i);
    start(t);
    pause(1000/i+5);
    stop(t);
end
