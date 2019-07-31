fclose all
clear all
close all
clc

% Change input parameters!!!
gif_filename = 'outputs/test-grid.gif';
figure_title = 'Reaching error';
folder = 'datasets/exp-005-2/';
gif_frame_delay = .3;
loop_count = Inf;
slide_points = 100:100:500;

% Tree
nImages = numel(slide_points);

% Prepare data for gif
idx = 1;
for i = slide_points
    filename = strcat(folder, 'data-', string(i), '.txt');
    fig = draw_grid(filename, figure_title, i);

    frame = getframe(fig);
    im{idx} = frame2im(frame);    
    close(fig);
    idx = idx + 1;
end

% Generate gif file
% https://www.mathworks.com/help/matlab/ref/imwrite.html#btv452g-1
for idx = 1:nImages
    [A,map] = rgb2ind(im{idx}, 256);
    if idx == 1
        imwrite(A,map,gif_filename,'gif','LoopCount',loop_count,'DelayTime',gif_frame_delay);
    else
        imwrite(A,map,gif_filename,'gif','WriteMode','append','DelayTime',gif_frame_delay);
    end
end
