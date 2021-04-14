function [x_g, y_g, theta_w] = fwd_kin(q1, q5, q_wrist)
% Calculates forward kinematics of robot
% @param q1 angle of motor 1
% @param q5 angle of motor 2
% @return x, y, and theta of wrist joint

    global l1; global l2; global l3; global l4; global l5;
    global xg; global yg;

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
    
    theta_w= pi/2;
    
    % rot = [cos(theta_w), -sin(theta_w);...
    %        sin(theta_w), cos(theta_w)];
       
    % g = [x + xg, y + yg]';
    % g = rot*g;
    
    % x_g = g(1);
    % y_g = g(2);
    
    x_g = x + xg*cos(theta_w);
    y_g = y + yg*sin(theta_w);
end