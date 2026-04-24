function [Fa, Fr] = VFF(currentPose, targetPose, map, searchWindowSize, scale, origin)
    
    Fcr = 2;
    Fca = 1;

    targetX = targetPose(1);
    targetY = targetPose(2);

    currentX = round(currentPose(1)*scale + origin);
    currentY = round(currentPose(2)*scale + origin);

    targetDeltaX = targetX - currentPose(1);
    targetDeltaY = targetY - currentPose(2);

    targetDistance = sqrt(targetDeltaY^2 + targetDeltaX^2);
    targetDirection = [targetDeltaX, targetDeltaY]./targetDistance;

    Fa = Fca.*targetDirection;
    
    % Calcular Força repulsiva
    Fr = [0, 0];

    range = floor(searchWindowSize/2);

    [mHeight, mWidth] = size(map);

    x_min = max(1, currentX-range);
    x_max = min(mHeight, currentX+range);
    y_min = max(1, currentY-range);
    y_max = min(mWidth, currentY+range);

    for x=x_min:x_max
        for y=y_min:y_max
            
            Cxy = map(x, y);
            
            if Cxy == 0
                continue;
            end
            
            deltaX = x - currentX;
            deltaY = y - currentY;
            distance = sqrt(deltaX^2 + deltaY^2);

            if distance == 0
                continue;
            end

            intensity = Fcr*Cxy/distance^2;
            direction = [deltaX, deltaY]./distance;
            Fr = Fr - intensity*direction;
            
        end
    end
end