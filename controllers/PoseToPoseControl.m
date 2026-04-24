function PoseToPoseControl(tbot, params)

    r = rateControl(params.rate);
    trajectory = zeros(params.maxIterations, 3);

    target.x = params.target(1);
    target.y = params.target(2);
    target.theta = params.target(3);

    for it=1:params.maxIterations

        [pose.x, pose.y, pose.theta, pose.timestamp] = tbot.readPose();
    
        pose.theta = normalizeAngle(pose.theta);

        deltaTheta = normalizeAngle(target.theta - pose.theta);
        trajectory(it,:) = [pose.x, pose.y, pose.theta];

        % Gets the error of the robot orientation related to the target
        alpha = getAngularError(pose, target);
        
        beta = target.theta - pose.theta - alpha;          % Final orientation error
        beta = normalizeAngle(beta);

        % Gets the robot's linear distance from the target position
        rho = getEuclidianDistance(pose, target);       

        % Goal verification
        if ((rho < params.toleranceErrorDist) && (abs(deltaTheta) < params.toleranceErrorAngle)) 
            tbot.stop(); 
            break; 
        end
        
         % Control law
        linearVelocity = params.kpRho*rho;
        angularVelocity = params.kpAlpha*alpha + params.kpBeta*beta;

        % Velocity saturation
        linearVelocity = max(min(linearVelocity, params.vMax), -params.vMax);
        angularVelocity = max(min(angularVelocity, params.wMax), -params.wMax);

        tbot.setVelocity(linearVelocity, angularVelocity);

        waitfor(r);    
    
    end
end