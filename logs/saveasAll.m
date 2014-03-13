function saveasAll(filename, fontsize, axis_array)
    % Saves the current figure as a .eps, .png, and .fig
    %
    % @param filename name before the extension to save
    % @param fontsize (optional) specify the plot's font size (often useful
    %   for embedding in papers
    % @param (Optional) axis_array array of axes to change the font size on
    %   useful for multi-axis plots (such as plotyy)
        
    if (nargin >= 2)
      
        if (nargin < 3)
            axis_array(1) = gca;
        end
        
        for i=1:length(axis_array)
            xlhand = get(axis_array(i),'xlabel');
            set(xlhand, 'fontsize', fontsize);

            ylhand = get(axis_array(i),'ylabel');
            set(ylhand, 'fontsize', fontsize);

            thand = get(axis_array(i),'title');
            set(thand, 'fontsize', fontsize);

            set(axis_array(i), 'FontSize', fontsize);
        end
    end

    saveas(gcf, strcat(filename, '.png'));
    saveas(gcf, strcat(filename, '.fig'));
    saveas(gcf, strcat(filename, '.eps'), 'epsc');
end