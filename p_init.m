% Initialization file for the main project Simulink model
% contains constants and calculates required parameters

% Motor: DCX 14 L (24V)
% motor paramters from datasheet 

V_nom = 24;             % nominal voltage (V)
w_0 = 7740*(2*pi/60);   % no load angular velocity (rad/s)
i_0 = 9.2 / 1000;       % no load current (A)
tq_stall = 18.9/1000;   % stall torque (Nm)
i_stall = 0.647;        % stall current (A)

R_term = 37.1;          % terminal resistance (Ohms)
L_term = 1.61 / 1000;   % terminal inductance (H)
 
K_m = 29.2 / 1000;      % torque (motor) constant (Nm/A)
K_b = 1/(327*2*pi/60);  % back EMF constant ( V/(rad/s) )

J_r = 0.939 * 1e-7;     % rotor interia (kg * m^2)
J_a = 0.00018;          % arm interia (kg * m^2)

% main motor gearing
gear_ratio = 35;        % gearing ratio (1:x)
gear_eff = 0.8;         % gearing efficiency 

% calculate other required parameters

J_a = J_a/(gear_ratio^2);   % scaled arm intertia (Nm)
t_0 = K_m * i_0;            % no load torque, due to friction (Nm)
B_r = t_0/w_0;              % rotor friction (Nm*s/rad)


% amplifier constants
% K_p = 2.3560;
% a = 2.7e-14;
% b = 2.6e-07;
K_p = 4.72e6/1.34e6;
a = 0;
b = 1/1.34e6;

v_max_amp = +V_nom;   % maximum votlage output by amp at 5V input (V)
v_min_amp = -V_nom;   % minimum votlage output by amp at 0V input (V)

% encoder
enc_res = 0.875 * pi/180;   % (rad/state) 

% IRQ frequency and computational time
comp_time = 0.000679456 * 1.25; % approx 1 millisecond

% define object positions
bin = [-2, 5];
pos_mm = [-8.0, 15;...
          -2.0, 15;...
           4.0, 15];

% generate trajectory vectors for repeating sequences
start_point = bin;
finish_point = pos_mm(3,:);
time_steps = 50;

% get time to complete trajectory in seconds
stop_time = str2num(get_param('controller_test', 'StopTime'));
%stop_time = 0.75
total_time = stop_time + stop_time/time_steps;     

traj = create_traj(start_point, finish_point, 50);
traj = cat(1, traj, flip(traj));   % concatenate reverse path 
x_vect = traj(:,1)';
y_vect = traj(:,2)';
time_vect = 0:(total_time/(2*time_steps-1)):(total_time);

% all three
dispose_all = 1;    % 1 will run path for all three marshmallows
if dispose_all == 1
    total_time = 0.75;
    traj = [];
    for i = 1:3
         % set endpoint for current marshmallow
        finish_point = pos_mm(i,:);

        % create ith trajectory
        ith_traj = create_traj(start_point, finish_point, 50);
        ith_traj = cat(1, ith_traj, flip(ith_traj)); % concatenate reverse path 

        % concatenate to complete trajectory
        traj = cat(1, traj, ith_traj);
    end
    % create sequence vectors
    x_vect = traj(:,1)';
    y_vect = traj(:,2)';
    time_vect = 0:(total_time/(6*time_steps-1)):(total_time);
end

% PID constants for MAIN motors
P_m = 2.84; 
I_m = 0.00;
D_m = 0.00;
