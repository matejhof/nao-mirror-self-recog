clc
clear all
close all

grid_size = 10;
fileID = fopen('datasets/exp-005-2/data-100.txt','r');
formatSpec = '%f';
A = fscanf(fileID,formatSpec);
A = reshape(A, [5, numel(A) / 5])';
fclose(fileID);

res_i = find(A(:,3) ~= 9999);

% Get skin coordinates
formatSpec = '%f';
skin_filename = 'skin-coordinates/torso.txt';
fileID = fopen(skin_filename,'r');
skin = fscanf(fileID, formatSpec);
skin = reshape(skin, [7, numel(skin) / 7])';
fclose(fileID);

% Grid edges
edges = [];
for j = 0:9
    for i = 1:9
        edges = [edges; j*10+i j*10+i+1];
    end
end
for i = 1:10
    for j = 0:8
        edges = [edges; j*10+i (j+1)*10+i];
    end
end
hold on

%scatter3(A(:,1),A(:,2),A(:,3));
scatter(A(:,1),A(:,2), 'r', 'filled')
%scatter(A(res_i,3),A(res_i,4), 'b', 'filled')
scatter(skin(:,2),skin(:,3), 'b', 'filled');

% Plot grid
i = 1;
plot([A(edges(i,1),1) A(edges(i,2),1)], [A(edges(i,1),2) A(edges(i,2),2)], 'r');
plot([A(edges(i,1),3) A(edges(i,2),3)], [A(edges(i,1),4) A(edges(i,2),4)], 'b');
for i = 1:size(edges,1)
    plot([A(edges(i,1),1) A(edges(i,2),1)], [A(edges(i,1),2) A(edges(i,2),2)], 'Color', [1,.7,.7]);
end
edges = [
    %1 2; 2 3; 3 4; 4 5; 5 6; 6 7; 7 8; 8 9; 9 10;
    %1 11; 11 21; 21 31; 31 41; 41 51; 51 61; 61 71; 71 81; 81 91; 
    1 5; 1 6; 2 6; 2 7; 3 7; 3 8; 4 8; 4 9;
    5 10; 6 11; 7 12; 8 13; 9 14;
    10 15; 11 15; 11 16; 12 16; 12 17; 13 17; 13 18; 14 18;
    15 19; 16 20; 17 21; 18 22;
    19 23; 20 23; 20 24;21 24; 21 25; 22 25
];
for i = 1:size(edges,1)
	plot([skin(edges(i,1),2) skin(edges(i,2),2)], [skin(edges(i,1),3) skin(edges(i,2),3)], 'Color', [.7,.7,1]);
end

scatter(A(:,1),A(:,2), 'r', 'filled')
scatter(skin(:,2),skin(:,3), 'b', 'filled');
legend('Target goals', 'Skin taxels') %, 'Location', 'southeast');

% title('Incidence graph')
xlabel('X [m]')
ylabel('Y [m]')
title('Target goal grid')

% Reaching error mean and std
reaching_mean = mean(A(res_i,5))
reaching_std = std(A(res_i,5))
