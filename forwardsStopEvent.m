% Event Function to Stop Integration
function [value, isterminal, direction] = forwardsStopEvent(t, x, vlc)
    % COMMENT TODO %
    vlc_values = interp1(vlc(:, 1), vlc(:, 2), x(1), "linear");
    %F_values = interp1(F(:, 1), F(:, 2), x(1), "linear");

    % Stop if F crossed or L = U 
    value = [x(1) - 1; x(2) - vlc_values];
    % Stop integration when either condition is met
    isterminal = [1; 1];
    % Detect any crossing
    direction = [0; 0];  
end