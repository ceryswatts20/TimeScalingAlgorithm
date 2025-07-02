% Event Function to Stop Integration
function [value, isterminal, direction] = backwardsStopEvent(t, x, vlc)
    % COMMENT TODO %
    values = interp1(vlc(:, 1), vlc(:, 2), x(1), "linear");

    % Stop if s = 0 or L = U
    value = [x(1); x(2) - values];
    % Stop integration when either condition is met
    isterminal = [1; 1];
    % Detect any crossing
    direction = [0; 0];
end