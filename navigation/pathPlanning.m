function path = pathPlanning(mapName, savePath, robot_radius, scale)

    arguments
        mapName
        savePath     = []
        robot_radius = 0.105
        scale        = 20        
    end

    % Prepare map
    map          = loadMap(mapName);
    robot_pixels = round(robot_radius * scale);
    se           = strel('disk', robot_pixels);
    map          = imdilate(map, se);

    [start, goal] = selectStartGoal(map);
    path          = aStar(map, start, goal);
    showPath(map, path, start, goal);

    if ~isempty(savePath)
        save(savePath, 'path');
    end
end