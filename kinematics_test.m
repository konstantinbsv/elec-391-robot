% Contains constants and calculations for all kinematics
% related aspects of the robot
clc
% arm lengths (cm)
global l1; global l2; global l3; global l4; global l5;
global xg; global yg;

l1 = 6.5; l4 = l1;     % arms attached to motors
l2 = 8.5; l3 = l2;     % arms attached to gripper
l5 = 4;                % motor spacing, 5th link
xg = 0;                % x offset from q3 to gripper center 
yg = 3.5;              % y offset from q3 to gripper center 

test_kin = 1;

center = -l5/2;
% use a repeating sequence for trajectory

f2 = figure();
axis([center-10 center+10 0 20]);
title('Robot Workspace');
xlabel('x (cm)');
ylabel('y (cm)')
axis square
hold all


% draw marshmallows
d = 3;          % diamter of marshmallows
spacing = 2;    % cm from edge to edge
c_to_c = spacing * d; % distance from center to center

init_x = (-l5/2 - c_to_c) + xg;
init_y = 15;
pos_mm = [[0, 0]; [0,0]; [0,0]];     % position of inital mm
for i = 1:3
    pos_mm(i,1) = init_x + c_to_c.*(i-1);
    pos_mm(i,2) = init_y;
    viscircles(pos_mm(i,:), d/2);
end

% draw disposal bin
bin_s = 5;              % bin side length
bin_x = center+xg;      % bin center x
bin_y = init_y - 10;    % bin center y
rectangle('Position', [bin_x-bin_s/2, bin_y-bin_s/2,...
                       bin_s, bin_s]);


if test_kin == 1
    %for q1 = pi/6:(pi/40):pi/2
    for yy = flip(5:15)
        %for q5 = pi/10+q1*0.8 :(pi/40): pi/3+q1*1.05
        d = -0.6*yy + 9;    % is 0 at max and 6 an min y
        for xx = (-8+d):0.5:(4-d)
            [q1, q5] = inv_kin(xx, yy);
            %[x_cur, y_cur, theta_w] = fwd_kin(q1, q5, pi/2);
            res = fwd_kin_vect(q1, q5, pi/2);
            x_cur = res(1); y_cur = res(2); theta_w = res(3);
            
            plot(x_cur, y_cur, '.')
            
            % plot arms
            plot_arms(q1, q5, x_cur, y_cur);
            
            % check inverse kinematics
            [q1_calc, q5_calc] = inv_kin(x_cur, y_cur);
            disp(['q1 actual=', num2str(q1), ' q1 calc=', num2str(q1_calc), newline,...
                'q5 actual=', num2str(q5), ' q5 calc=', num2str(q5_calc)]);


            % compare inverse kinematics to actual input angle to 3 decimal
            % places
            if (round(q1*100) == round(q1_calc*100)); (disp('PASS')); 
            else; (disp('FAIL')); end

            pause(0.01)
        end
    end
else % test trajectory generation
    % create trajectory
    %traj = create_traj([-8 8], [3 8], 50);
    for mm = 1:3
        traj = create_traj([bin_x bin_y], pos_mm(mm,:), 40);
        pause(1)
        for i = 1:length(traj)
        % extract x and y from trajectory
        x_cur = traj(i,1);
        y_cur = traj(i,2);
        
        % use inverse kinematics to calculate motor positions from 
        % trajectory coordinates
        [q1_calc, q5_calc] = inv_kin(x_cur, y_cur);
        
        % plot point
        plot(x_cur, y_cur, '.');
        
        % plot arms
        plot_arms(q1_calc, q5_calc, x_cur, y_cur);
        
        pause(0.01)
        end
    end
end
    
function plot_arms(q1, q5, g_x, g_y)
global l1; global l2; global l3; global l4; global l5;
global xg; global yg;

    %  position of wrist
    x = g_x - xg;
    y = g_y - yg;

    % arm 1
    x_1 = l1*cos(q1);
    y_1 = l1*sin(q1);
    delete(findall(gcf,'Tag','a1'));
    line([0 x_1], [0 y_1],'Color','red','LineStyle','-', 'tag', 'a1');

    % arm 2
    delete(findall(gcf,'Tag','a2'));
    line([x_1 x], [y_1 y],'Color','green','LineStyle','-', 'tag', 'a2');

    % arm 4
    q5_p = q5 - pi/2;
    x_2 = -l4*sin(q5_p) - l5;
    y_2 = l4*cos(q5_p);

    delete(findall(gcf,'Tag','a4'));
    line([-l5 x_2], [0 y_2],'Color','blue','LineStyle','-', 'tag', 'a4');

    % arm 3
    delete(findall(gcf,'Tag','a3'));
    line([x_2 x], [y_2 y],'Color','magenta','LineStyle','-', 'tag', 'a3');
    
    % draw gripper y
    delete(findall(gcf,'Tag','yg'));
    line([x x], [y y+yg],'Color','magenta','LineStyle','-', 'tag', 'yg')
    
    % draw gripper x
    delete(findall(gcf,'Tag','xg'));
    line([x x+xg], [y+yg y+yg],'Color','magenta','LineStyle','-', 'tag', 'xg');
    
    
end