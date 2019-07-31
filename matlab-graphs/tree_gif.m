\fclose all
clear all
close all
clc

% Change input parameters!!!
gif_filename = 'outputs/exp-016-tree-torso.gif';
figure_title = 'Tree (SAGG-RIAC), 20 points max per region, torso & right hand';
folder = 'datasets/exp-016-2/';
skin_filename = 'skin-coordinates/torso.txt';
add_overlay = 0;
gif_frame_delay = .1;
slide_points = 100:10:1000;

% Get skin coordinates
formatSpec = '%f';
fileID = fopen(skin_filename,'r');
skin = fscanf(fileID, formatSpec);
skin = reshape(skin, [7, numel(skin) / 7])';

% Plot skin
i_min = 1;
i_max = size(skin, 1);
%scatter(skin(i_min:i_max,2),skin(i_min:i_max,3), 'r', 'filled')

% Tree
nImages = numel(slide_points);

% Get max interest over all slides
max_interest = 0;
for i = slide_points
    % Read file
    fileID = fopen(strcat(folder, 'tree-', string(i), '.txt'),'r');
    A = fscanf(fileID, formatSpec);
    A = reshape(A, [5, numel(A) / 5])';
    max_interest = max(max_interest, max(A(:,5)));
    fclose(fileID);
end

% Prepare data for gif
idx = 1;
for i = slide_points
    filename = strcat(folder, 'tree-', string(i), '.txt');
    fig = draw_tree(filename, figure_title, i, max_interest, true);
    
    %Draw overlays
    if add_overlay
        sc = scatter(skin(i_min:i_max,2),skin(i_min:i_max,3), 'w', 'filled');
        alpha(sc,.5);
    end
    
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
        imwrite(A,map,gif_filename,'gif','LoopCount',Inf,'DelayTime',gif_frame_delay);
    else
        imwrite(A,map,gif_filename,'gif','WriteMode','append','DelayTime',gif_frame_delay);
    end
end
