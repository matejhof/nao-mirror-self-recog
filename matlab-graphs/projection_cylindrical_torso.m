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

% Plot cylinder
z_min = -50 * scaleFactor;
z_max = 150 * scaleFactor;
num_points = 200;
cylinder_r = 100 * scaleFactor;
[x,y,z]=cylinder(cylinder_r, num_points);
z(1,:) = z_min;
z(2,:) = z_max;
handle=surf(x(:,1:num_points/4),y(:,1:num_points/4),z(:,1:num_points/4),'FaceColor', 'blue', 'FaceAlpha', 0.3, 'EdgeAlpha', 0.0);
handle=surf(x(:,num_points*3/4:end),y(:,num_points*3/4:end),z(:,num_points*3/4:end),'FaceColor', 'blue', 'FaceAlpha', 0.3, 'EdgeAlpha', 0.0);

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
%skin(:,1) = skin(:,1) - 5;
%skin(:,2) = skin(:,2);
%skin(:,3) = skin(:,3) + 132;
taxel_size = 200;
scatter3(skin(:,1), skin(:,2), skin(:,3), taxel_size, 'b.');

% Plot projection lines
projections = [];
proj_z = 50 * scaleFactor;
for i = 1:size(skin,1)
    taxel_x = skin(i,1);
    taxel_y = skin(i,2);
    taxel_z = skin(i,3);

    % Axis-projection
    v = [taxel_x taxel_y];
    v = v / norm(v) * cylinder_r;
    %line = plot3([0,v(1)], [0,v(2)], [taxel_z,taxel_z], 'r');
    %projections = [projections; [v taxel_z]];
    
    % Center-projection
    v2 = [v(1) v(2) 0];
    u2 = [taxel_x taxel_y (taxel_z - proj_z)];
    cos_val = dot(v2,u2)/(norm(u2)*norm(v2));
    length = cylinder_r / cos_val;
    v = u2 / norm(u2) * length;
    v(3) = proj_z+v(3);
    line = plot3([0,v(1)], [0,v(2)], [proj_z,v(3)], 'r');
    projections = [projections; v];
    %line.Color = [1 0 0 .5];
end

% Plot 
scatter3(projections(:,1), projections(:,2), projections(:,3), taxel_size, 'r.');

% Plot skin edges
edges = [
    1 5; 1 6; 2 6; 2 7; 3 7; 3 8; 4 8; 4 9;
    5 10; 6 11; 7 12; 8 13; 9 14;
    10 15; 11 15; 11 16; 12 16; 12 17; 13 17; 13 18; 14 18;
    15 19; 16 20; 17 21; 18 22;
    19 23; 20 23; 20 24;21 24; 21 25; 22 25
];
for i = 1:size(edges,1)
    %plot3([projections(edges(i,1),1) projections(edges(i,2),1)], [projections(edges(i,1),2) projections(edges(i,2),2)], [projections(edges(i,1),3) projections(edges(i,2),3)], 'k');
end

% Figure metas
%title({"Central projection of artificial skin taxels"; "on robot's torso onto a cylindrical surface"});
set(gcf, 'Position',  [100, 100, 500, 600]);
