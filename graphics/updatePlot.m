% function updatePlot(handles, traj, pose, target)
%     set(handles.traj,   'XData', traj(:,1),  'YData', traj(:,2));
%     set(handles.robot,  'XData', pose.x,      'YData', pose.y);
%     set(handles.target, 'XData', target.x,    'YData', target.y);
%     drawnow;
% end

function updatePlot(handles, traj, pose, target, h, alpha)
    set(handles.traj,   'XData', traj(:,1), 'YData', traj(:,2));
    set(handles.robot,  'XData', pose.x,    'YData', pose.y);
    set(handles.target, 'XData', target.x,  'YData', target.y);

    % Desenha histograma polar à volta do robô
    if nargin >= 5 && ~isempty(h)
        numSectors = length(h);
        hNorm      = h / (max(h) + 1e-6) * 0.3;   % normaliza para max 0.3m de raio
        angles     = (0:numSectors-1) * alpha - pi + alpha/2;

        % Coordenadas polares → cartesianas, fechando o polígono
        hx = pose.x + hNorm .* cos(angles);
        hy = pose.y + hNorm .* sin(angles);
        hx = [hx, hx(1)];
        hy = [hy, hy(1)];

        set(handles.hist, 'XData', hx, 'YData', hy);
    end

    drawnow;
end