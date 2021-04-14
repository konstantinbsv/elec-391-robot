function traj = create_traj(start, final, steps)
% Given a set of start/stop points, returns a calculated trajectory
% @param start start point 2D vector
% @param final final point 2D vector
% @param steps number of time steps from start to end 
% @return positions vector of positions
    
    x0 = 0;     % inital x
    xf = 1;     % final x
    xd0 = 0;    % initial velocity
    xdf = 0;    % final velocity
    
    % calculate smooth tracjectory guiding function
    % quintic polynomial
    
    % create vector of time steps
    t = (0:steps-1)';
    tf = max(t);        % get final time
    
    % create matrix from boundary conditions
    X = [0      0       0       0       0       1;
        tf^5    tf^4    tf^3    tf^2    tf      0;
        0       0       0       0       1       0;
        5*tf^4  4*tf^3  3*tf^2  2*tf    1       0;
        0       0       0       2       0       0;
        20*tf^3 12*tf^2 6*tf    2       0       0];
    
    % calculate coefficients
    coeffs = (X \ [x0 xf xd0 xdf 0 0]')';
    
    % get coefficients of derivatives 
    coeffs_d = coeffs(1:5) .* (5:-1:1);
    coeffs_dd = coeffs_d(1:4) .* (4:-1:1);
    
    % evaluate polynomial at t's
    s = polyval(coeffs, t);
    
    traj = double.empty(steps,0);
    for i = 0:length(s)
        traj = (1 - s)*start + s*final;
    end
end