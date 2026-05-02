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
        [pose.x, pose.y, pose.theta] = tbot.readPose();
        pose.theta = normalizeAngle(pose.theta);
        
        xRobotGrid = round(pose.x * params.scale) + params.origin;
        yRobotGrid = round(pose.y * params.scale) + params.origin;

        % --- LIDAR ---
        [~, lddata] = tbot.readLidar();
        vidx  = tbot.getInRangeLidarDataIdx(lddata);
        scans = lddata.Cartesian(vidx, :);

        [x_occ, y_occ]  = processScan(pose, scans, params);
        
        logOddsMap = bayesianMapUpdate(xRobotGrid, yRobotGrid, x_occ, y_occ, logOddsMap, params);

        %Convert to probability 
        probMap = 1 ./ (1 + exp(-logOddsMap));

        % Update display 
        updatePlot(handles, probMap, xRobotGrid, yRobotGrid, pose.theta);

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

function [x_occ, y_occ] = processScan(pose, scans, params)

    maxRange = 3.5; % LIDAR max range (assignment)
    minRange = 0.1; 

    xScans = scans(:,1);
    yScans = scans(:,2);

    ranges = sqrt(xScans.^2 + yScans.^2);
    valid = ~isnan(ranges) & ~isinf(ranges) & (ranges < maxRange) & (ranges > minRange);
    
    xScans = xScans(valid);
    yScans = yScans(valid);

    % Transform to world
    xWorld = pose.x + xScans*cos(pose.theta) - yScans*sin(pose.theta);
    yWorld = pose.y + xScans*sin(pose.theta) + yScans*cos(pose.theta);

    x_occ = round(xWorld * params.scale) + params.origin;
    y_occ = round(yWorld * params.scale) + params.origin;

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
    set(handles.img,   'CData', map');
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