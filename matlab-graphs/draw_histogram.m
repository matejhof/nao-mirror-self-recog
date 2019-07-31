clc
clear all
close all

total_taxels = 24;
% fileID = fopen('mb-data-2.txt','r');
fileID = fopen('datasets/exp-001/data.txt','r');
formatSpec = '%f';
A = fscanf(fileID,formatSpec);
A = reshape(A, total_taxels + 1, numel(A) / (total_taxels + 1))';
fclose(fileID);

B = (sum(A(:,2:end) ~= 0, 2) ./ total_taxels)';
bar(A(:,1)', B);
yaxis([0 1]);

xaxis([0 A(end,1) + 10 ]);
xticks( 0:100:A(end,1) );
%xticklabels( A(:,1)' );

title('Progress of skin discovery with motor babbling');
xlabel('Iteration number');
ylabel('Skin coverage');
