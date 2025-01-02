my_data = resreader(['C:\Users\fanny\Desktop\Schwimmwinkelschaetzer\Simulation\BEAR24_v20240206_step.res']);
time = my_data.System_Time;

speed = my_data.System_Mass_Center_Speed;
sensor_speed_x = [time, my_data.Sensor_Vx];
sensor_speed_y = [time, my_data.Sensor_Vy];
sensor_yawrate = [time, my_data.Vehicle_States_yaw_angle];
sideslip = [time, my_data.Vehicle_Side_Slip_Angle];
steering1 = [time, my_data.Wheel_Spindle_Steer_L1];
steering2 = [time, my_data.Wheel_Spindle_Steer_R1];
diff_front_right = [time, my_data.differential_front_wheel_front_right_omega];
diff_front_left= [time, my_data.differential_front_wheel_front_left_alpha];
diff = [time, my_data.differential_front_wheel_front_right_angle];
xpo = [time, my_data.Vehicle_CM_Global_X];
ypo = [time, my_data.Vehicle_CM_Global_Y];
% bicycle model parameters provided by GreanBEAR member Nils Haage

m_veh = 215.000; %[kg] vehicle mass
m_drv = 75.000; % [kg] driver mass
m = m_veh + m_drv; % [kg] overall mass
L = 1.570; %[m] wheel base
lv =0.874; %[m] distance from center of mass to front axle
lh = L-lv; %[m] distance from center of mass to rear axle
Cy_f = 13223; %[N/rad] front tire corner stiffness
Cy_r = 13223; %[N/rad] rear tire corner stiffness
Thetaz = 124.403600; %[kg*m^2] yaw polar inertia


v = 18/3.6;


% Definition Matrix A
a11 = -(Cy_f + Cy_r) / (m * v);
a121 = m * pow2(v);
a122 = Cy_r * lh - Cy_f * lv;
a12 = ((a122) / a121) - 1 ;
a21 = (Cy_r * lh - Cy_f * lv) / Thetaz;
a221 = Cy_r * pow2(lh) + Cy_f * pow2(lv);
a22 = -(a221) / Thetaz * v;

A = [a11 a12; a21 a22];

% Definition Matrix B
b1 = Cy_f / (m * v);
b2 = Cy_f * lv / Thetaz;

B = [b1; b2];

% Messmatrix 
H = [1 0;0 1]; 
Q = [0.8 2; 1 0.8]; % Prozessrauschen
R = 0.0001; % Messrauschen