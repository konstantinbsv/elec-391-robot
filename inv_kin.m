function [q1, q5] = inv_kin(g_x, g_y)
% Calculates inverse kinematics of robot
% @param x position of gripper center
% @param y position of gripper center
% @return q1, q2 of motors

    global l1; global l2; global l3; global l4; global l5;
    global xg; global yg;
    
    % calculate position of wrist
    x = g_x - xg;
    y = g_y - yg;
    
    % find vector lengths to wrist joint
    p = sqrt(x^2 + y^2);
    r = sqrt((x+l5)^2 + y^2);
    
    % calculate angles to p and r vectors from x-axis
    alpha = acos(x/p);
    beta = acos((x+l5)/r);
    
    % use law of cosines to find inner angle of triangles
    q1_p = acos((p^2 + l1^2 - l2^2) / (2*p*l1));
    q5_p = acos((r^2 + l4^2 - l3^2) / (2*r*l4));
    
    % find q1 and q5 motor angles
    q1 = alpha - q1_p;
    q5 = beta + q5_p;
    
end