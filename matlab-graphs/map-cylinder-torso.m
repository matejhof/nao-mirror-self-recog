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
%view([135 35]);
view([105 5]);

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
skin_filename = 'skin-coordinates/torso-highres.txt';
fileID = fopen(skin_filename,'r');
skin = fscanf(fileID, formatSpec);
skin = reshape(skin, [3, numel(skin) / 3])';
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

% Figure metas
%title({"Central projection of artificial skin taxels"; "on robot's torso onto a cylindrical surface"});
%set(gcf, 'Position',  [100, 100, 500, 600]);

% Generate skin map
skin_map = [];
for i = 1:size(projections, 1)
    proj_x = projections(i,1);
    proj_y = projections(i,2);
    proj_z = projections(i,3);

    v2 = [proj_x proj_y];
    u2 = [cylinder_r 0];
    angle = acos(dot(v2,u2)/(norm(u2)*norm(v2)));
    x = cylinder_r * angle * sign(proj_y);
    y = proj_z;

    % Planar projection - for comparison
    %x = proj_y;
    %y = proj_z;
    
    skin_map = [skin_map; x y];
end

% Output skin map
output_filename = 'outputs/highres-torso.txt';
fid = fopen(output_filename,'wt');
for ii = 1:size(skin_map,1)
    fprintf(fid,'%g\t',skin_map(ii,:));
    fprintf(fid,'\n');
end
fclose(fid);

% Display skin map
figure
scatter(skin_map(:,1), skin_map(:,2));
xaxis(-0.15, 0.15);
