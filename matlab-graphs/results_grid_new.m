clc
clear all
close all

% Plot skin
formatSpec = '%f';
skin_filename = 'outputs/highres-torso.txt';
fileID = fopen(skin_filename,'r');
skin = fscanf(fileID, formatSpec);
skin = reshape(skin, [2, numel(skin) / 2])';
fclose(fileID);

scatter(skin(:,1),skin(:,2), 'b', 'filled');
hold on

% Grid edges
edges = [];

% Targets
targets = [3 6 14 19 24 29 32 57 64 69 71 101 114 127 158 170 186 193 199 205 209 212 227 234 239 241];
scatter(skin(targets,1), skin(targets,2), 'r', 'filled');

edges = [
    % Horizontal
    6 14; 14 69; 69 24; 24 32;
    57 19; 19 64; 64 29; 29 71;
    101 158; 158 114; 114 170; 170 127;
    186 193; 193 239; 239 205; 205 212;
    227 199; 199 234; 234 209; 209 241;
    % Vertical
    6 57; 57 101; 101 186; 186 227;
    14 19; 19 158; 158 193; 193 199;
    69 64; 64 114; 114 239; 239 234;
    24 29; 29 170; 170 205; 205 209;
    32 71; 71 127; 127 212; 212 241
];
for i = 1:size(edges,1)
	plot([skin(edges(i,1),1) skin(edges(i,2),1)], [skin(edges(i,1),2) skin(edges(i,2),2)], 'Color', [1,.7,.7]);
end

% Plot targets once again (needed for proper legend)
scatter(skin(targets,1), skin(targets,2), 'r', 'filled');

legend('Skin taxels', 'Target goals') %, 'Location', 'southeast');

% title('Incidence graph')
xlabel('X [m]')
ylabel('Y [m]')
title('Target goal grid')

