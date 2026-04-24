function [start, goal] = selectStartGoal(map)
    figure;
    showMap(map, 'Click START (green) then GOAL (blue)');

    disp('Click START point');
    [x1, y1] = ginput(1);
    start = [round(y1), round(x1)];
    plot(start(2), start(1), 'go', 'MarkerFaceColor', 'g');

    disp('Click GOAL point');
    [x2, y2] = ginput(1);
    goal = [round(y2), round(x2)];
    plot(goal(2), goal(1), 'bo', 'MarkerFaceColor', 'b');

    pause(1);
    close;
end