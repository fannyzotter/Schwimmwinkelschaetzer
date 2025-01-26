my_data = resreader(['C:\Users\fanny\Desktop\Schwimmwinkelschaetzer\Simulation\BEAR24_v20240206_crc.res']);
time = my_data.System_Time;

speed = [time, my_data.System_Mass_Center_Speed];
acceleration = [time, my_data.Vehicle_States_longitudinal_acc_wrt_road];
sensor_speed_x = [time, my_data.Sensor_Vx];
sensor_speed_y = [time, my_data.Sensor_Vy];
sensor_yawangle = [time, my_data.Vehicle_States_yaw_angle];
sensor_yawrate = [time, my_data.Sensor_YawRate_gyro];
sensor_omega1 = [time, my_data.Wheel_Omega_R1];
sensor_omega2 = [time, my_data.Wheel_Omega_R2];
sensor_omega = (sensor_omega1 + sensor_omega2)/2;

sideslip = [time, my_data.Vehicle_Side_Slip_Angle];
steering_left = my_data.Wheel_Spindle_Steer_L1;
noise = 0.03 * randn(size(steering_left));
noisy_steering1 = [time, noise+steering_left];
steering2 = [time, my_data.Wheel_Spindle_Steer_R1];
steering1 = [time, my_data.Wheel_Spindle_Steer_L1];
diff_front_right = [time, my_data.differential_front_wheel_front_right_omega];
diff_front_left= [time, my_data.differential_front_wheel_front_left_alpha];
diff = [time, my_data.differential_front_wheel_front_right_angle];
xpo = [time, my_data.Vehicle_CM_Global_X];
ypo = [time, my_data.Vehicle_CM_Global_Y];
torque = [time, my_data.motor_to_rear_left_wheel_torque];
% bicycle model parameters provided by GreanBEAR member Nils Haage

m_veh = 215.000; %[kg] vehicle mass
m_drv = 75.000; % [kg] driver mass
m = m_veh + m_drv; % [kg] overall mass
L = 1.570; %[m] wheel base
lv =0.874; %[m] distance from center of mass to front axle
lh = L-lv; %[m] distance from center of mass to rear axle
Cy_f = 13223; %[N/rad] front tire corner stirearwheelffness
Cy_r = 13223; %[N/rad] rear tire corner stiffness
Thetaz = 124.403600; %[kg*m^2] yaw polar inertia
Fz = 2900;
gamma = 0;
Rw = 0.26; % Reifenradius [m]

%v = 18/3.6;


%P = ones(9,9);

    Q = [
    0.000001,    0,       0,       1e-2,    0,       1e-8,    0,  0,       0;
    0,       0.000001,    0,       1e-2,    0,       1e-8,    0,  0,       0;
    0,       0,        1e-8,    1e-4,       0,       1e-8,    0,  0,       0;
    0.1,     0.1,     0.1,     0.1,         0.1,     0.1,     0.1,0.1,     0.1;
    0,       0,       0,       1e-4,       1e-8,     1e-8,    0,  0,       0;
    0,       0,       0,       0,          0,        1e-8,    0,  0,       0;
    0,       0,       0,       0.02,       0.02,     1e-8,    0,  0,       0;
    0,       0,       0,       1e-4,       0.02,     1e-8,    0,  0,       0;
    0.1,     0.1,     0.1,     0.1,         0.1,     0.1,     0.1,0.1,     0.1;
];


    Q = diag([0.02, 0.02, 0.01, 0.1, 0.03, 0.00001, 0.00001, 0, 0.0005]);
    
sensor_x0 = [xpo(1,2);
        ypo(1,2);
        steering1(1,2);
        speed(1,2)/3.6;
        sensor_yawangle(1,2);
        sensor_yawrate(1,2);
        sideslip(1,2);
        0;
        sensor_omega(1,2);];
