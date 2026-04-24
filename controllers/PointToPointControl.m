function PointToPointControl(tbot, params)

    r = rateControl(params.rate);
    trajectory = zeros(params.maxIterations, 3);
    target.x = params.target(1);
    target.y = params.target(2);

    for it=1:params.maxIterations

        [pose.x, pose.y, pose.theta, pose.timestamp] = tbot.readPose();
    
        pose.theta = normalizeAngle(pose.theta);

        trajectory(it,:) = [pose.x, pose.y, pose.theta];

        % Gets the error of the robot orientation related to the target
        deltaTheta = getAngularError(pose, target);
        
        % Gets the robot's linear distance from the target position
        distance = getEuclidianDistance(pose, target); 
    
        % if the linear distance is above the tolerance error, it means the robot
        % reached the target position, so it can stop
        if distance < params.toleranceError
           tbot.stop(); 
           break; 
        end
    
        % Proportional control
        linearVelocity = params.kV*distance;
        angularVelocity = params.kW*deltaTheta;

        tbot.setVelocity(linearVelocity, angularVelocity);

        waitfor(r);    
    
    end
end