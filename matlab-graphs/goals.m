clc
clear all
close all

% Get skin coordinates
formatSpec = '%f';
skin_filename = 'skin-coordinates/torso.txt';
fileID = fopen(skin_filename,'r');
skin = fscanf(fileID, formatSpec);
skin = reshape(skin, [7, numel(skin) / 7])';
fclose(fileID);

% Get goals
formatSpec = '%f';
goal_filename = 'datasets/misc/goals-tree.txt';
fileID = fopen(goal_filename,'r');
goals = fscanf(fileID, formatSpec);
goals = reshape(goals, [2, numel(goals) / 2])';
fclose(fileID);

figure
hold on
grid on

edges = [
    %1 2; 2 3; 3 4; 4 5; 5 6; 6 7; 7 8; 8 9; 9 10;
    %1 11; 11 21; 21 31; 31 41; 41 51; 51 61; 61 71; 71 81; 81 91; 
    1 5; 1 6; 2 6; 2 7; 3 7; 3 8; 4 8; 4 9;
    5 10; 6 11; 7 12; 8 13; 9 14;
    10 15; 11 15; 11 16; 12 16; 12 17; 13 17; 13 18; 14 18;
    15 19; 16 20; 17 21; 18 22;
    19 23; 20 23; 20 24;21 24; 21 25; 22 25
];

scatter(goals(:,1),goals(:,2), [], [.7,.7,.7], 'filled');
scatter(skin(:,2),skin(:,3), 'b', 'filled');
for i = 1:size(edges,1)
	plot([skin(edges(i,1),2) skin(edges(i,2),2)], [skin(edges(i,1),3) skin(edges(i,2),3)], 'Color', [.7,.7,1]);
end
scatter(skin(:,2),skin(:,3), 'b', 'filled');
legend('Goals', 'Artificial skin taxels')


x_min = min(skin(:,2));
x_max = max(skin(:,2));
y_min = min(skin(:,3));
y_max = max(skin(:,3));
dx = x_max - x_min;
dy = y_max - y_min;

xlim([x_min-dx x_max+dx])
ylim([y_min-dy y_max+dy])

%xlabel('X [m]')
%ylabel('Y [m]')
xticks([x_min-dx x_min x_max x_max+dx])
yticks([y_min-dy y_min y_max y_max+dy])
%xticklabels({'x_{min}-\Delta{x}', 'x_{min}', 'x_{max}', 'x_{max}+\Delta{x}'})
%yticklabels({'y_{min}-\Delta{y}', 'y_{min}', 'y_{max}', 'y_{max}+\Delta{y}'})
%title('Distribution of goals over discretized observation space')
title('Distribution of goals in SAGG-RIAC algorithm')
