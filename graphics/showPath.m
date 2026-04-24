function showPath(map, path, start, goal)
    figure;
    showMap(map, 'Planned Path');
    hold on;

    if ~isempty(path)
        plot(path(:,2), path(:,1), 'r-', 'LineWidth', 2);
    end

    plot(start(2), start(1), 'go', 'MarkerFaceColor', 'g');
    plot(goal(2),  goal(1),  'bo', 'MarkerFaceColor', 'b');
end