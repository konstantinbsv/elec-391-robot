% Initialization file for the main project Simulink model (WRIST)
% contains constants and calculates required parameters

% Motor: DCX 8 M (6V)
% motor paramters from datasheet 

V_nom_w = 12;              % nominal voltage (V)
w_0_w = 12900*(2*pi/60);  % no load angular velocity (rad/s)
i_0_w = 2.74 / 1000;      % no load current (A)
tq_stall_w = 1.13/1000;  % stall torque (Nm)
i_stall_w = 0.13;        % stall current (A)

R_term_w = 92.2;          % terminal resistance (Ohms)
L_term_w = 0.276 / 1000;  % terminal inductance (H)
 
K_m_w = 8.71 / 1000;      % torque (motor) constant (mNm/A)
K_b_w = 1/(1100*2*pi/60); % back EMF constant ( V/(rad/s) )

J_r_w = 0.035 * 1e-7;     % rotor interia (kg * m^2)
J_a_w = 0.0000515;        % arm interia (kg * m^2)


% wrist gearing
gear_ratio_w = 36;        % gearing ratio (1:x)
gear_eff_w = 0.76;         % gearing efficiency 

% calculate other required parameters
J_a = J_a/(gear_ratio_w^2);         % scaled arm intertia (Nm)
t_0_w = K_m_w * i_0_w;            % no load torque, due to friction (Nm)
B_r_w = t_0_w/w_0_w;              % rotor friction (Nm*s/rad)


% amplifier constants
% K_p = 2.3560;
% a = 2.7e-14;
% b = 2.6e-07;
K_p_w = 4.72e6/1.34e6;
a_w = 0;
b_w = 1/1.34e6;

v_max_amp_w = +V_nom_w;   % maximum votlage output by amp at 5V input (V)
v_min_amp_w = -V_nom_w;  % minimum votlage output by amp at 0V input (V)


% define wrist angles relative to arm 3 (deg)
ang_bin = 84.5 * pi/180;        % angle when over bin
ang_mm = [25, 40, 30].*pi/180;  % angle at marshmallow

       
% wrist kinematics
wrist_angle = [];
for sel_mm = 1:3 % marshmallows to dispose
    start_point_w = ang_bin;
    finish_point_w = ang_mm(sel_mm);
    time_steps_w = 50;
    
    % create forward and reverse trajectory angles
    ith_wrist_angle = create_traj([start_point_w, 0], [finish_point_w, 0], time_steps_w);
    ith_wrist_angle = ith_wrist_angle(:,1)'; % create vector
    ith_wrist_angle = cat(2, ith_wrist_angle, flip(ith_wrist_angle));
    
    % concatenate with complete path
    wrist_angle = cat(2, wrist_angle, ith_wrist_angle);
end



% PID constants for WRIST motor
P_w = 0.55;
I_w = 0;
D_w = 0.00;
