function [return_figure, reaching_mean, reaching_std] = draw_grid(filename, figure_title, iteration)
    grid_size = 10;
    unit_coefficient = 1000; % [m] -> [mm]
    units = "[mm]";

    fileID = fopen(filename, 'r');
    formatSpec = '%f';
    A = fscanf(fileID,formatSpec);
    A = reshape(A, [5, numel(A) / 5])';
    fclose(fileID);

    res_i = find(A(:,3) ~= 9999);
    A(:,1:2) = A(:,1:2) .* unit_coefficient;
    A(res_i,3:4) = A(res_i,3:4) .* unit_coefficient;

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

    return_figure = figure;
    hold on

    % Plot grid
    i = 1;
    plot([A(edges(i,1),1) A(edges(i,2),1)], [A(edges(i,1),2) A(edges(i,2),2)], 'b');
    plot([A(edges(i,1),1) A(edges(i,2),1)], [A(edges(i,1),2) A(edges(i,2),2)], 'r');
    %plot([A(edges(i,1),3) A(edges(i,2),3)], [A(edges(i,1),4) A(edges(i,2),4)], 'b');
    for i = 1:size(edges,1)
        plot([A(edges(i,1),1) A(edges(i,2),1)], [A(edges(i,1),2) A(edges(i,2),2)], 'r');
    end
    for i = 1:size(A,1)
        if (A(i,3) < 9999) && (A(i,4) < 9999)
            plot([A(i,1) A(i,3)], [A(i,2) A(i,4)], 'b');
        end
    end

    scatter(A(:,1),A(:,2), 'r', 'filled')
    scatter(A(res_i,3),A(res_i,4), 'b', 'filled')
    legend('Reached positions', 'Target grid', 'Location', 'southeast');
    xlabel({strcat("X ", units); strcat("i=", string(iteration))});
    ylabel(strcat("Y ", units));
    title(figure_title);

    % Reaching error mean and std
    reaching_mean = mean(A(res_i,5));
    reaching_std = std(A(res_i,5));
