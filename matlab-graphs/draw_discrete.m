function return_figure = draw_discrete(filename, figure_title, iteration, max_interest, min_x, min_y, max_x, max_y, side_size, show_iteration_number)
    step_x = (max_x - min_x) / side_size;
    step_y = (max_y - min_y) / side_size;

    cmap_ticks = 256;
    cmap = parula(cmap_ticks);

    return_figure = figure;
    hold on
    %set(gcf, 'color', 'w'); % Set figure background color to white
    colormap(parula);
    cb = colorbar('Ticks',[0, 1], 'TickLabels', [sprintf("%.1f", 0.0), sprintf("%.1f", 1.0)]);
    ylabel(cb, 'Normalized measure of interest in each region');

    % Read file
    formatSpec = '%f';
    fileID = fopen(filename, 'r');
    progress = fscanf(fileID, formatSpec);
    progress = reshape(progress, [side_size, side_size])';
    for ii = 1:side_size
        for jj = 1:side_size
            color_index = round((cmap_ticks - 1) * progress(ii,jj) / max_interest) + 1;
            x1 = min_x + step_x * (ii - 1);
            x2 = x1 + step_x;
            y1 = min_y + step_y * (jj - 1);
            y2 = y1 + step_y;
            fill([x1 x1 x2 x2], [y1 y2 y2 y1], cmap(color_index,:));
        end
    end
    title(figure_title);
    %annotation('textbox',[.2 .5 .3 .3],'String',strcat('i=',string(iteration)),'FitBoxToText','on');
    ylabel('y [m]');
    xlbl = {'x [m]'};
    if show_iteration_number
        xlbl{2} = strcat('Iteration: ',string(iteration));
    end
    xlabel(xlbl);
    fclose(fileID);
end