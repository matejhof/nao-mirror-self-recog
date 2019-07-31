fclose all
clear all
close all
clc

%compare_dirs = ["datasets/exp-017/" "datasets/exp-018/" "datasets/exp-012/" "datasets/exp-013/" "datasets/exp-016-2/"];
%labels = ["Random motor babbling" "Random goal babbling" "Discretized Progress, 31x31 grid" "Discretized Progress, 15x15 grid" "Tree (SAGG-RIAC), 20 points max per region"];

compare_dirs = ["datasets/exp-017/" "datasets/exp-018/" "datasets/exp-019-1/" "datasets/exp-019-2/" "datasets/exp-021/"];
labels = ["Random motor babbling" "Random goal babbling" "Discretized Progress, 15x15 grid" "Discretized Progress, 32x32 grid" "Tree (SAGG-RIAC), 10 points per region"];

%data_range = 100:100:1000;
data_range = 10:10:500;
formatSpec = '%f';

figure
hold on
grid on
xlabel('Iteration number')
ylabel('Mean reaching distance [cm]')
title('Comparison of exploration strategies, torso & right hand')
% title('Comparison of DOF, head')

for dir = compare_dirs
    means = [];
    stds = [];
    skin_portion = [];
    
    for i = data_range
        filename = strcat(dir, 'data-', string(i), '.txt');
        fileID = fopen(filename,'r');
        data = fscanf(fileID, formatSpec);
        data = reshape(data, [5, numel(data) / 5])';
        data_subset = data(find(data(:,5) ~= 9999), 5) .* 100;
        
        [i numel(data_subset)]
        
        reaching_mean = mean(data_subset);
        reaching_std = std(data_subset);
        reaching_skin_portion = numel(data_subset) / numel(data(:,5));
        
        means = [means, reaching_mean];
        stds = [stds, reaching_std];
        skin_portion = [skin_portion, reaching_skin_portion];
    end

    %errorbar(data_range, means, stds, 'LineWidth', 1, 'CapSize', 6);
    plot(data_range, means, 'LineWidth', 2);
end

legend(labels)
axis([10 inf 0 inf])
set(gcf, 'Position',  [100, 100, 800, 400]);

fclose all;
