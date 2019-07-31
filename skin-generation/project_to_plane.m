clear all
close all
clc

formatSpec = '%f';

fileID = fopen('_output-triangle-2-map.txt','r');
A = fscanf(fileID, formatSpec);
A = reshape(A, [3, numel(A) / 3])';

%fileID = fopen('coordinates/head.txt','r');
%A = fscanf(fileID, formatSpec);
%A = reshape(A, [7, numel(A) / 7])';

hold on
% axis off

edges = [
    1 5; 1 6; 2 6; 2 7; 3 7; 3 8; 4 8; 4 9;
    5 10; 6 11; 7 12; 8 13; 9 14;
    15 10; 15 11; 16 11; 16 12; 17 12; 17 13; 18 13; 18 14;
    19 15; 20 16; 21 17; 22 18;
    23 19; 23 20; 24 20; 24 21; 25 21; 25 22;
];

for i = 1:size(edges,1)
    % plot([A(edges(i,1),2) A(edges(i,2),2)], [A(edges(i,1),3) A(edges(i,2),3)], 'b')
end

%scatter3(A(:,1),A(:,2),A(:,3));
i_min = 1;
i_max = size(A, 1);
for i = i_min:i_max
    scatter(A(i_min:i_max,2),A(i_min:i_max,3), 'r', 'filled')
    break
    scatter(A(i,2),A(i,3), 'b', 'filled')
    title(strcat('Current point: ', int2str(i)))
    pause(0.5)
end
