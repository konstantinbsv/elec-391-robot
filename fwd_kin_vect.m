function res = fwd_kin(q1, q5, q_wrist)
% Calculates forward kinematics of robot
% @param q1 angle of motor 1
% @param q5 angle of motor 2
% @return x, y, and theta of wrist joint

    l1 = 6.5; l4 = l1;     % arms attached to motors
    l2 = 8.5; l3 = l2;     % arms attached to gripper
    l5 = 4;                % motor spacing, 5th link
    xg = 0;                % x offset from q3 to gripper center 
    yg = 3.5;                % y offset from q3 to gripper center

    % calculate q5 complimentary angle
    %q5_p = q5 - pi/2;
    
    % system of nonlinear equations used for finding passifve joint angles
    % qj(1) = q2_p and qj(2) = q4_p
    function F = joint_angles(qj)
        F(1) =  l1*cos(q1) - l2*cos(qj(1)) - l3*cos(qj(2)) - l4*cos(q5) + l5;  % x coord 
        F(2) = l1*sin(q1) + l2*sin(qj(1)) - l3*sin(qj(2))  - l4*sin(q5);       % y coord
    end

    % numerical solve system of nonlinear equations
    ja_hangle = @joint_angles;
    qj_0 = [pi/2, pi/2];  % starting points
    options = optimset('Display','off'); % suppress output
    qj_s = fsolve(ja_hangle, qj_0, options);
    q2_p = qj_s(1);
    q4_p = qj_s(2);
    
    % find x using solutions
    x = l1*cos(q1) - l2*cos(q2_p);
    y = l1*sin(q1) + l2*sin(q2_p);
    
    %theta_w= q4_p + q_wrist;
    theta_w = pi/2;
    
%     rot = [cos(theta_w), -sin(theta_w);...
%            sin(theta_w), cos(theta_w)];
%        
%     g = [x + xg, y + yg]';
%     
%     g = rot*g;
    
    res(1) = x + xg*cos(theta_w);
    res(2) = y + yg*sin(theta_w);
    res(3) = theta_w;
end