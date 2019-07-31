fclose all
clear all
close all
clc

% Change input parameters!!!
gif_filename = 'outputs/exp-012-discrete-torso-with-taxels.gif';
figure_title = 'Goal Babbling with Discretized Progress, torso & right hand';
folder = 'datasets/exp-012/';
skin_filename = 'skin-coordinates/torso.txt';
add_overlay = 1;
gif_frame_delay = .1;
slide_points = 100:50:1000;

% Get skin coordinates
formatSpec = '%f';
fileID = fopen(skin_filename,'r');
skin = fscanf(fileID, formatSpec);
skin = reshape(skin, [7, numel(skin) / 7])';

min_x = min(skin(:,2));
max_x = max(skin(:,2));
min_y = min(skin(:,3));
max_y = max(skin(:,3));
dx = (max_x - min_x);
dy = (max_y - min_y);
discrete_min_x = min_x - dx;
discrete_min_y = min_y - dy;
discrete_max_x = max_x + dx;
discrete_max_y = max_y + dy;

% Plot skin
i_min = 1;
i_max = size(skin, 1);
%scatter(skin(i_min:i_max,2),skin(i_min:i_max,3), 'r', 'filled')

% Discrete
nImages = numel(slide_points);

% Get max interest over all slides
max_interest = 0;
for i = slide_points
    % Read file
    fileID = fopen(strcat(folder, 'discrete-', string(i), '.txt'),'r');
    A = fscanf(fileID, formatSpec);
    side_size = sqrt(numel(A));
    A = reshape(A, [side_size, side_size])';
    max_interest = max(max_interest, max(max(A)));
end

% Prepare data for gif
idx = 1;
for i = slide_points
    filename = strcat(folder, 'discrete-', string(i), '.txt');
    fig = draw_discrete(filename, figure_title, i, max_interest, discrete_min_x, discrete_min_y, discrete_max_x, discrete_max_y, side_size, true);
    
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