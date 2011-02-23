%gs = 1
tstart = tic;
for i = 1:1000
    Panel_com('dump_frame', [384, 48, 1, 0, round(rand(1,384)*255)]);
end
telapsed = toc(tstart);
frameRate = 1000/telapsed

%gs = 2
tstart = tic;
for i = 1:1000
    Panel_com('dump_frame', [768, 48, 2, 0, round(rand(1,768)*255)]);
end
telapsed = toc(tstart);
frameRate = 1000/telapsed

%gs = 3
tstart = tic;
for i = 1:1000
    Panel_com('dump_frame', [1152, 48, 3, 0, round(rand(1,24*48)*255)]);
end
telapsed = toc(tstart);
frameRate = 1000/telapsed

%gs = 4
tstart = tic;
for i = 1:1000
    Panel_com('dump_frame', [1536, 48, 4, 0, round(rand(1,1536)*255)]);
end
telapsed = toc(tstart);
frameRate = 1000/telapsed