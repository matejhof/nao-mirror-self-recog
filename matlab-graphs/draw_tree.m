function return_figure = draw_tree(filename, figure_title, iteration, max_interest, show_iteration_number)
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
    progress = reshape(progress, [5, numel(progress) / 5])';
    max_interest = max(progress(:,5));
    for ii = 1:size(progress, 1)
        color_index = round((cmap_ticks - 1) * progress(ii,5) / max_interest) + 1;
        x1 = progress(ii,1);
        x2 = progress(ii,3);
        y1 = progress(ii,2);
        y2 = progress(ii,4);
        fill([x1 x1 x2 x2], [y1 y2 y2 y1], cmap(color_index,:));
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
