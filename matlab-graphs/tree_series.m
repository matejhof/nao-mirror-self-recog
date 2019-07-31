fclose all
clear all
close all
clc

% Change input parameters!!!
figure_title = {'SAGG-RIAC exploration strategy, 20 points per region'; 'Body parts: torso, right hand'};
folder = 'datasets/exp-016-2/';
skin_filename = 'skin-coordinates/torso.txt';
add_overlay = 1;
output_format = 'epsc'; % png / epsc
slide_points = [160 300 500 800 1000];

% Get skin coordinates
formatSpec = '%f';
fileID = fopen(skin_filename,'r');
skin = fscanf(fileID, formatSpec);
skin = reshape(skin, [7, numel(skin) / 7])';

% Plot skin
i_min = 1;
i_max = size(skin, 1);

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
    fig = draw_tree(filename, figure_title, i, max_interest, false);

    %Draw overlays
    if add_overlay
        sc = scatter(skin(i_min:i_max,2),skin(i_min:i_max,3), 'w', 'filled');
        alpha(sc,.5);
    end

    saveas(gcf,char(strcat('outputs/tree-slide-', string(i))),output_format)
    close(fig)
end
