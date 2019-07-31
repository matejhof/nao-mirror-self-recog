fclose all
clear all
close all
clc

% Change input parameters!!!
figure_title = {'Discretized progress exploration strategy, 31x31 grid'; 'Body parts: torso, right hand'};
folder = 'datasets/exp-012/';
skin_filename = 'skin-coordinates/torso.txt';
add_overlay = 1;
output_format = 'png'; % png / epsc
slide_points = 100:50:1000;
save_slides = [100 200];

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
for i = save_slides
    filename = strcat(folder, 'discrete-', string(i), '.txt');
    fig = draw_discrete(filename, figure_title, i, max_interest, discrete_min_x, discrete_min_y, discrete_max_x, discrete_max_y, side_size, false);

    %Draw overlays
    if add_overlay
        sc = scatter(skin(i_min:i_max,2),skin(i_min:i_max,3), 'w', 'filled');
        alpha(sc,.5);
    end

    % Does not work for some reason
    %saveas(gcf,char(strcat('outputs/discrete-slide-', string(i))),output_format)
    %close(fig)
    %break
end
