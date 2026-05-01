function mapBuilding(tbot, params, savePath)

    % MAP INIT
    logOddsMap = zeros(params.mapSize, params.mapSize);
    probMap    = 0.5 * ones(params.mapSize, params.mapSize);

    % CONTROL
    global v w exitFlag;
    v = 0; 
    w = 0;
    exitFlag = false;

    % PLOT 
    handles = setupPlot(probMap);
    r = rateControl(params.rate);

    % ================= MAIN LOOP =================
    while ~exitFlag

        % --- Robot pose ---
        [xRobot, yRobot, theta] = tbot.readPose();
        theta = normalizeAngle(theta);

        % --- LIDAR ---
        [~, lddata] = tbot.readLidar();
        vidx  = tbot.getInRangeLidarDataIdx(lddata);
        scans = lddata.Cartesian(vidx, :);

        maxRange = 3.5; % LIDAR max range (assignment)
        minRange = 0.1; 
    
        xScans = scans(:,1);
        yScans = scans(:,2);
    
        % Filter invalid / too far
        ranges = sqrt(xScans.^2 + yScans.^2);
        valid = ~isnan(ranges) & ~isinf(ranges) & (ranges < maxRange) & (ranges > minRange);
    
        xScans = xScans(valid);
        yScans = yScans(valid);
    
        % Transform to world
        xWorld = xRobot + xScans*cos(theta) - yScans*sin(theta);
        yWorld = yRobot + xScans*sin(theta) + yScans*cos(theta);
    
        % Convert to grid 
        xScaled = round(xWorld * params.scale) + params.origin;
        yScaled = round(yWorld * params.scale) + params.origin;
    
        xRobotGrid = round(xRobot * params.scale) + params.origin;
        yRobotGrid = round(yRobot * params.scale) + params.origin;

        logOddsMap = bayesianMapUpdate(xRobotGrid, yRobotGrid, xScaled, yScaled, logOddsMap, params);

        %Convert to probability 
        probMap = 1 ./ (1 + exp(-logOddsMap));
        % Update display 
        updatePlot(handles, probMap, xRobotGrid, yRobotGrid, theta);

        % Move robot 
        tbot.setVelocity(v, w);

        waitfor(r);
    end

    % SAVE 
    binaryMap = probMap > 0.5;

    imwrite(uint8(flipud(1 - probMap) * 255), savePath + ".png");
    save(savePath + ".mat", 'probMap', 'binaryMap', 'logOddsMap');

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