function export_light_pdf(f, filename)
    % find every graphics object with a Color property
    objs = findall(f, '-property', 'Color');
    origColors = get(objs, 'Color');

    % define your dark->light color remap here
    remap = containers.Map();
    remap('1_1_1')       = [0 0 0];        % white -> black (payload dot)
    remap('0.9_0.9_0.9') = [0.3 0.3 0.3];  % light gray pendulum -> dark gray

    for k = 1:numel(objs)
        c = get(objs(k), 'Color');
        key = sprintf('%.3g_%.3g_%.3g', c(1), c(2), c(3));
        if isKey(remap, key)
            set(objs(k), 'Color', remap(key));
        end
    end

    theme(f, 'light');                      % flips axes/bg/grid/text
    exportgraphics(f, filename, ...
        'BackgroundColor', 'white', ...
        'ContentType', 'vector');           % true vector PDF
    theme(f, 'dark');                       % restore your app theme

    % restore original hardcoded colors
    for k = 1:numel(objs)
        set(objs(k), 'Color', origColors{k});
    end
end