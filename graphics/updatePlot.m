% function updatePlot(handles, traj, pose, target)
%     set(handles.traj,   'XData', traj(:,1),  'YData', traj(:,2));
%     set(handles.robot,  'XData', pose.x,      'YData', pose.y);
%     set(handles.target, 'XData', target.x,    'YData', target.y);
%     drawnow;
% end

function updatePlot(handles, traj, pose, target, h, alpha)
    % Map (if handle exists and map was passed)
    set(handles.traj,   'XData', traj(:,1), 'YData', traj(:,2));
    set(handles.robot,  'XData', pose.x,    'YData', pose.y);
    set(handles.target, 'XData', target.x,  'YData', target.y);

    % VFH histogram
    if nargin >= 5 && ~isempty(h)
        numSectors = length(h);
        hNorm  = h / (max(h) + 1e-6) * 0.3;
        angles = (0:numSectors-1) * alpha - pi + alpha/2;
        hx = [pose.x + hNorm .* cos(angles), pose.x + hNorm(1) * cos(angles(1))];
        hy = [pose.y + hNorm .* sin(angles), pose.y + hNorm(1) * sin(angles(1))];
        set(handles.hist, 'XData', hx, 'YData', hy);
    end

    drawnow limitrate;   % limitrate is better than plain drawnow for real-time loops
end