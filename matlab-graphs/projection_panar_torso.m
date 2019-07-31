clear all
close all
clc

% Create figure
figure
hold on

% Plot robot
scaleFactor = .001;
file_torso = 'torso_casing.stl';
torso = stlread(file_torso);

torso.vertices = torso.vertices * scaleFactor;
patch(torso,'FaceColor',       [0.8 0.8 1.0], ...
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
z_min = -40 * scaleFactor;
z_max = 140 * scaleFactor;
plane_x = 100 * scaleFactor;
surf([plane_x plane_x], [-100*scaleFactor 100*scaleFactor], [z_min z_max; z_min z_max],'FaceColor', 'blue', 'FaceAlpha', 0.3, 'EdgeAlpha', 0.0);

% Get skin coordinates
formatSpec = '%f';
skin_filename = 'skin-coordinates/torso.txt';
fileID = fopen(skin_filename,'r');
skin = fscanf(fileID, formatSpec);
skin = reshape(skin, [7, numel(skin) / 7])';
fclose(fileID);

% Plot axis
% plot3([0, 0], [0, 0], [z_min, z_max], 'Color', 'r');

% Plot skin
skin(:,1) = skin(:,1);
skin(:,2) = skin(:,2);
skin(:,3) = skin(:,3);
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
    1 5; 1 6; 2 6; 2 7; 3 7; 3 8; 4 8; 4 9;
    5 10; 6 11; 7 12; 8 13; 9 14;
    10 15; 11 15; 11 16; 12 16; 12 17; 13 17; 13 18; 14 18;
    15 19; 16 20; 17 21; 18 22;
    19 23; 20 23; 20 24;21 24; 21 25; 22 25
];
for i = 1:size(edges,1)
    %plot3([plane_x plane_x], [skin(edges(i,1),2) skin(edges(i,2),2)], [skin(edges(i,1),3) skin(edges(i,2),3)], 'k');
end


% Figure metas
%title({"Parallel projection of artificial skin taxels"; "on robot's torso onto a plane"});
set(gcf, 'Position',  [100, 100, 500, 600]);
