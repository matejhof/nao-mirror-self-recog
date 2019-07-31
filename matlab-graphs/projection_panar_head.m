clear all
close all
clc

% Create figure
figure
hold on

% Plot robot
scaleFactor = .001;
file_head = 'head_casing.stl';
head = stlread(file_head);

head.vertices = (rotz(-7 / 180 * pi) * head.vertices')';
head.vertices = head.vertices * scaleFactor;
patch(head,'FaceColor',       [0.8 0.8 1.0], ...
           'EdgeColor',       'none',        ...
           'FaceLighting',    'gouraud',     ...
           'AmbientStrength', 0.15, ...
           'FaceAlpha',       0.3 );

% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');

% Fix the axes scaling, and set a nice view angle
axis('image');
view([135 35]);
%view([105 5]);

% Plot plane
z_min = 110 * scaleFactor;
z_max = 250 * scaleFactor;
plane_x = 100 * scaleFactor;
surf([plane_x plane_x], [-100*scaleFactor 100*scaleFactor], [z_min z_max; z_min z_max],'FaceColor', 'blue', 'FaceAlpha', 0.3, 'EdgeAlpha', 0.0);

% Get skin coordinates
formatSpec = '%f';
skin_filename = 'skin-coordinates/head.txt';
fileID = fopen(skin_filename,'r');
skin = fscanf(fileID, formatSpec);
skin = reshape(skin, [7, numel(skin) / 7])';
fclose(fileID);

% Plot axis
% plot3([0, 0], [0, 0], [z_min, z_max], 'Color', 'r');

% Plot skin
skin(:,1) = skin(:,1) - 5*scaleFactor;
skin(:,2) = skin(:,2);
skin(:,3) = skin(:,3) + 132*scaleFactor;
taxel_size = 200;
scatter3(skin(:,1), skin(:,2), skin(:,3), taxel_size, 'b.');

% Plot projections
scatter3(plane_x * ones(size(skin(:,1))), skin(:,2), skin(:,3), taxel_size, 'r.');

% Plot projection lines
for i = 1:size(skin,1)
    taxel_x = skin(i,1);
    taxel_y = skin(i,2);
    taxel_z = skin(i,3);

    line = plot3([0,plane_x], [taxel_y,taxel_y], [taxel_z,taxel_z], 'r');
end

% Plot skin edges
edges = [
    1 2; 2 3; 3 4; 4 5;
    6 7; 1 7; 7 8; 8 9; 3 9; 9 10; 10 11; 11 12; 5 11;
    13 14; 14 15; 15 16; 16 17;
    18 19; 13 19; 19 20; 20 21; 15 21; 21 22; 22 23; 23 24; 17 23;
    6 18; 8 20; 10 22; 12 24
];
for i = 1:size(edges,1)
    %plot3([plane_x plane_x], [skin(edges(i,1),2) skin(edges(i,2),2)], [skin(edges(i,1),3) skin(edges(i,2),3)], 'k');
end

% Figure metas
%title({"Parallel projection of artificial skin taxels"; "on robot's head onto a plane"});
set(gcf, 'Position',  [100, 100, 500, 600]);
