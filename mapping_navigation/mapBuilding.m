function mapBuilding(tbot, params, savePath)

    params.scale   = 20;
    params.origin  = 175;
    params.mapSize = 350;

    map   = zeros(params.mapSize, params.mapSize);
    
    r = rateControl(params.rate);

    global v w exitFlag;
    v = 0; w = 0; exitFlag = false;

    handles = setupPlot(map);

    while ~exitFlag
        [xRobot, yRobot, theta] = tbot.readPose();
        theta = normalizeAngle(theta);

        [~, lddata] = tbot.readLidar();
        vidx        = tbot.getInRangeLidarDataIdx(lddata);
        scans       = lddata.Cartesian(vidx, :);
        xScans      = scans(:, 1);
        yScans      = scans(:, 2);
    
        % Robot frame -> world coordinates
        xWorld = xRobot + xScans*cos(theta) - yScans*sin(theta);
        yWorld = yRobot + xScans*sin(theta) + yScans*cos(theta);
    
        % World -> grid coordinates
        xScaled = round(xWorld * params.scale) + params.origin;
        yScaled = round(yWorld * params.scale) + params.origin;
    
        % Filter points outside map boundaries
        valid = xScaled > 0 & xScaled <= size(map, 2) & ...
                yScaled > 0 & yScaled <= size(map, 1);
    
        idx      = sub2ind(size(map), yScaled(valid), xScaled(valid));
        map(idx) = 1;

        xRobotGrid = round(xRobot * params.scale) + params.origin;
        yRobotGrid = round(yRobot * params.scale) + params.origin;
        
        disp(isvalid(handles.img));
        updatePlot(handles, map, xRobotGrid, yRobotGrid, theta);
        tbot.setVelocity(v, w);
        waitfor(r);
    end

    imwrite(uint8(flipud(1 - map) * 255), savePath + ".png");
    save(savePath + ".mat", 'map');

    close(handles.fig);
    tbot.stop();
end


function handles = setupPlot(map)
    handles.fig   = figure('KeyPressFcn', @keyboardHandle);
    handles.img   = imagesc(map);
    colormap(flipud(gray));
    set(gca, 'YDir', 'normal');
    axis equal;
    hold on;
    handles.robot = plot(0, 0, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    handles.arrow = quiver(0, 0, 0, 0, 'r', 'LineWidth', 2);
    
    % Confirma que tudo ficou na mesma figura
    disp(handles.img.Parent.Parent == handles.fig);
end

function updatePlot(handles, map, xR, yR, theta)
    arrowLength = 10;
    set(handles.img,   'CData', map);
    set(handles.robot, 'XData', xR,  'YData', yR);
    set(handles.arrow, 'XData', xR,  'YData', yR, ...
                       'UData', arrowLength * cos(theta), ...
                       'VData', arrowLength * sin(theta));
    drawnow limitrate;
end

function keyboardHandle(~, eventData)
    global v w exitFlag;
    linearUpdate  = 0.025;
    angularUpdate = 0.05;

    switch eventData.Key
        case 'rightarrow'; w = w - angularUpdate;
        case 'leftarrow';  w = w + angularUpdate;
        case 'uparrow';    v = v + linearUpdate;
        case 'downarrow';  v = v - linearUpdate;
        case 'space';      v = 0; w = 0;
        case 'escape';     v = 0; w = 0; exitFlag = true;
    end
end