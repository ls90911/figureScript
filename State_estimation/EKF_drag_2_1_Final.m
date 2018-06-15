close all;
clear all;

%Basement data sets with real vision data
%gate position at x = 4.2 y = 0.8 z = -1.4 (ned)
%uncomment the desired dataset for plotting

Flight_pos = cell(3,1) ;
Flight_time = cell(3,1);
%3 flights
for f = 1:3

%basement  1
if(f == 1)
% start = 100
% VarName = csvread('basement_flight_logs\second_session\flight_73_1lp.csv');
% start = 100
% VarName = csvread('basement_flight_logs\second_session\flight_81_1lp.csv');%good
start = 100
VarName = csvread('basement_flight_logs/second_session/flight_103_3lp.csv');%good

end

%maybe 2
if(f == 2)

% start = 100
% VarName = csvread('basement_flight_logs\second_session\flight_100_2lp_g2.csv');%good

start = 100
VarName = csvread('basement_flight_logs/second_session/flight_102_2lp_g2.csv');%good

end

% 3
if(f == 3)

%Good
% start = 100
% VarName = csvread('basement_flight_logs\second_session\flight_84_1lp_py01.csv');%good

%Best third
start = 100
VarName = csvread('basement_flight_logs/second_session/flight_101_almost_3.csv');%good

% start = 100
% VarName = csvread('basement_flight_logs\second_session\flight_81_1lp.csv');%good

% start = 100
% VarName = csvread('basement_flight_logs\second_session\flight_104_3lp_film.csv');%good

end

% stop = array_size(1);

counter = VarName(:,1); 
time_stamp = VarName(:,2);

%gyro scale 0.0139882 for deg/sec
gyro_scale = deg2rad(0.0139882);

%acc scale 0.0009766 for m/s2
acc_scale = 0.0009766;


gyro_p = VarName(:,3)*gyro_scale; gyro_q = VarName(:,4)*gyro_scale; gyro_r = VarName(:,5)*gyro_scale;
acc_x = VarName(:,6)*acc_scale; acc_y = VarName(:,7)*acc_scale; acc_z = VarName(:,8)*acc_scale;
phi = VarName(:,12); theta = VarName(:,13); psi = VarName(:,14);
pos_x = VarName(:,15); pos_y = VarName(:,16); pos_z = VarName(:,17);
vel_x = VarName(:,18); vel_y = VarName(:,19); vel_z = VarName(:,20);
shot_count = VarName(:,35);
corner_1_x = VarName(:,36); corner_1_y = VarName(:,37);
corner_2_x = VarName(:,38); corner_2_y = VarName(:,39);
corner_3_x = VarName(:,40); corner_3_y = VarName(:,41);
corner_4_x = VarName(:,42); corner_4_y = VarName(:,43);
sonar_alt = -VarName(:,17);

ls_pos_x = VarName(:,45);
ls_pos_y = VarName(:,46);
ls_pos_z = VarName(:,47);
	  
kf_pos_x = VarName(:,48);
kf_pos_y = VarName(:,49);
kf_vel_x = VarName(:,50);
kf_vel_y = VarName(:,51);
x_pos_hist = VarName(:,52);
y_pos_hist = VarName(:,53);
gate_quality = VarName(:,54);

acc_bias_x = VarName(:,55);
acc_bias_y = VarName(:,56);
acc_bias_z = VarName(:,57);
	  
local_x = VarName(:,58);
local_y = VarName(:,59);

gate_counter = VarName(:,60);

	 % //log_pid_error,
	 % //mean_speed,
arc_status_v_x_f = VarName(:,61);
log_pid_derror = VarName(:,62);
	  
primitive_in_use = VarName(:,63);
desired_phi = VarName(:,64);
log_phi_mean = movmean(VarName(:,64),100);
	  
side_1 = VarName(:,65);
side_2 = VarName(:,66);

%for latest flights
kf_pos_z = VarName(:,67);

  EKF_x = VarName(:,70);
  EKF_y = VarName(:,71);
  EKF_z = VarName(:,72);
  EKF_p = VarName(:,73);
  EKF_q = VarName(:,74);

  log_pid_error = VarName(:,75);
  
  vision_sample = VarName(:,76);

[data_size dummy ] = size(counter);

data_size

Flight_time{f} = time_stamp;


prev_sonar = 0;
%sonar spike removal
for n = 2:data_size
   if(sonar_alt(n)<0)
       sonar_alt(n) = sonar_alt(n-1);
   end
end

detect_count = 0;

prev_x_1 = 0;
prev_y_1 = 0;
prev_x_2 = 0;
prev_y_2 = 0;
prev_x_3 = 0;
prev_y_3 = 0;
prev_x_4 = 0;
prev_y_4 = 0;

gate_pos = [4.5 0.0 -1.8];
cam_pos = [0 0 0];%unknown dummy
[ gate_points ] = Calc_gate_points_order( gate_pos );

angle_position = [0 0 0];
LEAST_SQUERES_ERROR = [0 0 0];

%EKF 

pos_x_ls = zeros(data_size,1);
pos_y_ls = zeros(data_size,1);
pos_z_ls = zeros(data_size,1);

psi_offset = deg2rad(0);% 0.62;
%initialization
prev_time  = time_stamp(1);

 pos_nav = [0 0 0];
 pos_hold = [0 0 0];
 pos_out = [0 0 0];
 pos_out_time = 0;

 prev_pos = pos_nav;


%EKF init
EKF_count = 0;
HOLD_count = 0;
prev_EKF_time = 0;
EKF_time_vec = [0];
count = 0;

init_period = 5.5;%

bias_acc  = [0 0 0]';
bias_gyro = [0 0 0]';% 

X_dot = [0 0 0 0 0 0 0];

P_k_1 = zeros(7,7);
%P_k_1_k_1 = eye(7,7)*1;%initialize with original covariances
P_k_1_k_1 = [0.133245,-0.000095,-0.000940,-0.016218,0.039812,-0.000243,0.002118 ;
-0.000095,0.134346,0.001603,0.010960,0.000136,0.038623,0.002333 ;
-0.000940,0.001603,0.147673,0.191070,0.004705,0.004028,-0.052035 ;
-0.016218,0.010960,0.191070,1.761007,0.000836,0.025539,-0.303613 ;
0.039812,0.000136,0.004705,0.000836,0.177988,0.000282,-0.009567 ;
-0.000243,0.038623,0.004028,0.025539,0.000282,0.149500,0.011971 ;
0.002118,0.002333,-0.052035,-0.303613,-0.009567,0.011971,0.580125];

       P_k_1_k_1(1,1) = 1;
       P_k_1_k_1(2,2) = 1;
       P_k_1_k_1(3,3) = 1;
       P_k_1_k_1(4,4) = 1;
       P_k_1_k_1(5,5) = 1;
       P_k_1_k_1(6,6) = 1;
       P_k_1_k_1(7,7) = 1;

Q = diag([0.2 0.2 0.1 4.2 0.0 0.0 0.0]);

R_k = diag([1.10 0.50 0.5]);

% extract position from state prediction
H_k = [eye(3,3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3)];
DHx = eye(3);

%variables at ekf rate
acc_x_raw = [0];

Jac_mat_f = d_kf_calc_jacobian_43();%
Jac_mat_h = d_kf_calc_jacobian_h();

%turn logic
in_turn = zeros(data_size,1);
start_x = 0;
start_y = 0;
start_z = 0;

arc_pred = zeros(data_size,6);

time_stamp_filter = time_stamp(1:data_size);%

%matlab 3 coeficient radial distortion correction
load('calib_Flow_3c.mat');
x_p = 315-152.8;
y_p = 35.7;

%filtered accel
fc = 20; % Cut off frequency
fs = 512; % Sampling rate

[b,a] = butter(6,fc/(fs/2)); % Butterworth filter of order 6
acc_x_filter = filter(b,a,acc_x); 
acc_y_filter = filter(b,a,acc_y); 
acc_z_filter = filter(b,a,acc_z); 

detect_count = 0;
detect_count_h = 0;
detect_count_m = 0;
clearvars pos_vision_x pos_vision_y pos_vision_z time_stamp_m
detect_count_a = 0;

gate_d = 0;
gate_close = 0;

sum_norm = 0;

body_speed = zeros(data_size,3); 

X_int = zeros(data_size,7);
X_dot = zeros(data_size,7);

last_vision_update_t = 0;

%2d rotation test
local_coord_x = 3.5;
local_coord_y = 0.0;
gate_angle = deg2rad(45);
rot_2d = [cos(gate_angle) -sin(gate_angle);
          sin(gate_angle) cos(gate_angle)];

global_coords = rot_2d*[local_coord_x local_coord_y]';

figure()
plot(local_coord_x,local_coord_y,'+')
hold on
plot(global_coords(1),global_coords(2),'o')

%filtered position in global frame 
global_pos = zeros(data_size,3);
%number of gates
num_gates = 5;
%global x/y position pairs for each sequential gate
global_gate_frame_2d = zeros(num_gates,2);
%origin of local gate frame, at the end of a turn
global_x = 0;
global_y = 0;
local_frame_pos_x = [-0.5,7.0,2.25,1,-3.4,0];
local_frame_pos_y = [0,-1,-11,-7,-6.5,0];
psi_start = 0;

%primitive sequence
ARC_L = 1;
ARC_R = -1;
maneuvers = [ARC_L,ARC_L,ARC_R,ARC_L,ARC_L,ARC_L];

gate_initial_position_x = [5.3,9.8,2.8,2.8,3.5+0.8,10.8];
gate_initial_position_y = [0 ,-0.3 ,0 ,0 ,0 ,0 ];

turn_point = [10.5,10.5,10.5,10.5,10.5];
gate_initial_heading = [0.0/pi*180, -90.0/180*pi,-270.0/180*pi,-150.0/180*pi,-270.0/180*pi,0];
gate_altitude = [1.8,1.8,1.8,1.8,1.8,1.8];

arc_radius =     [1.0,             2.35,           1.0,		1,	 2, 2.5];
delta_arc_angle = [90.0/180*3.14,180.0/180*3.14,120.0/180*3.14,120.0/180*3.14, 90.0/180*3.14];

%psi test
psi_1 = zeros(data_size,1);
for i = 2:data_size
    psi_diff = psi(i)-psi(i-1);
    if(psi_diff < -pi)
        psi_1(i) = psi_1(i-1) + psi_diff + 2*pi;
    elseif(psi_diff > pi)
        psi_1(i) = psi_1(i-1) + psi_diff - 2*pi;
    else
        psi_1(i) = psi_1(i-1) + psi_diff;
    end
    %psi_1(i) = psi_diff;
end

    
%FLIGHT PROCESSING!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

%figure()
stop = data_size

%testing measurement timing
EKF_m_dt = zeros(data_size,1);
EKF_dt = 0;

%snapshot of filter
ss_state = 0;

X_int_prev = [0 0 0 0 0 0 0];

one_lap = 1;
stop_n = data_size;
%2 and 100 padding
for n = start:(stop-100)
    
    if(one_lap && gate_counter(n) < gate_counter(n-1))
        time_stamp(n);
        stop_n = n + 450;
        %break;
    end
    
    if(n>stop_n)
        break
    end
    
   if(primitive_in_use(n)-primitive_in_use(n-1) == 30)
       X_int_prev(4) = -1;
   end
    
   dt = time_stamp(n)-time_stamp(n-1);
   omega = [gyro_p(n) gyro_q(n)];
   
   body_speed(n,:) = (C_b_n(phi(n),theta(n),psi(n))'*[vel_x(n) vel_y(n) vel_z(n)]')';
 
   U_k = [EKF_x(n) EKF_y(n) EKF_z(n) EKF_p(n) EKF_q(n) phi(n) theta(n) 0];%psi(n)];
   
   %turning: assuming no lateral and vertical speed when entering turn
   
   if(in_turn(n-1) == 0 && primitive_in_use(n) == 21 && maneuvers(gate_counter(n)+1) == 1)% phi(n)>deg2rad(7) && gyro_r(n) > 0.6)
       %left turn
       in_turn(n) = 1;
       turn_radius = arc_radius(gate_counter(n)+1);
       
       start_x = global_pos(n-1,1);%
       start_y = global_pos(n-1,2);%
       start_z = global_pos(n-1,3);%
       psi_start = psi_1(n-1);
   elseif(in_turn(n-1) == 0 && primitive_in_use(n) == 21 && maneuvers(gate_counter(n)+1) == -1)%
       %right turn
       in_turn(n) = -1;
       turn_radius = arc_radius(gate_counter(n)+1);
       start_x = global_pos(n-1,1);%
       start_y = global_pos(n-1,2);%
       start_z = global_pos(n-1,3);%
       psi_start = psi_1(n-1);
   elseif(in_turn(n-1) ~= 0 && primitive_in_use(n) ~= 21)%
       %exit turn
       X_int_prev(1) = 0;%reset position estimation
       X_int_prev(2) = 0;
       in_turn(n) = 0;

   else
       in_turn(n) = in_turn(n-1);
   end
   
   if(in_turn(n) == 0)
       dot = d_kf_calc_f_nl_43_f(dt,X_int_prev,U_k);
   else
       dot = zeros(7,1);
   end
  
    X_dot(n,:) = dot';
  
   if(1&&in_turn(n) == 0 && time_stamp(n) - last_vision_update_t > 0.16)
        X_int(n,:) = X_int_prev + dot'*dt;
        X_int(n,3) = pos_z(n);
   else
        X_int(n,:) = X_int_prev + dot'*dt;
   end
   
  
  bounds = 0;
  if(bounds)
  % //bounding pos, speed and biases
  if(X_int(n,1) > 45)
      X_int(n,1) = 45;%//xmax
  end
  if(X_int(n,1) < -4)
      X_int(n,1) = -4;%//xmin
  end
  if(X_int(n,2) > 7)
      X_int(n,2) = 7;%//ymax
  end
  if(X_int(n,2) < -7)
      X_int(n,2) = -7;%//ymin
  end
  if(X_int(n,3) > 0)
      X_int(n,3) = 0;%//xmax
  end
  if(X_int(n,3) < -5)
      X_int(n,3) = -5;%//xmin
  end
  
  %//w speed
  if(X_int(n,4) > 1.5)
      X_int(n,4) = 1.5;%//ymax
  end
  if(X_int(n,4) < -1.5)
      X_int(n,4) = -1.5;%//ymin
  end
  
  %//acc biases
  if(X_int(n,5) > 1)
      X_int(n,5) = 1;%//ymax
  end
  if(X_int(n,5) < -1)
      X_int(n,5) = -1;%//ymin
  end
  if(X_int(n,6) > 1)
      X_int(n,6) = 1;%//ymax
  end
  if(X_int(n,6) < -1)
      X_int(n,6) = -1;%//ymin
  end
  if(X_int(n,7) > 2)
      X_int(n,7) = 2;%//ymax
  end
  if(X_int(n,7) < -2)
      X_int(n,7) = -2;%//ymin
  end
  end
   
   pos_nav = [pos_nav;X_int(1:3)];
   
   gate_pos = [gate_initial_position_x(gate_counter(n)+1) gate_initial_position_y(gate_counter(n)+1) -1.8];
   [ gate_points ] = Calc_gate_points_order( gate_pos );
   
   if(in_turn(n) == 1)
       if(pos_x(n)>2.5)
           %first turn
           arc_pred(n,1) = start_x + sin(psi(n))*turn_radius;%pos x
           arc_pred(n,2) = start_y + turn_radius -cos(psi(n))*turn_radius;%pos y
       else
           %second turn
           arc_pred(n,1) = start_x + sin(psi(n))*turn_radius;%pos x
           arc_pred(n,2) = start_y - turn_radius - cos(psi(n))*turn_radius;%pos y
       end
   else

   end

   %turn ended reset position and speed covariance
   if((in_turn(n)-abs(in_turn(n-1))) < 0)
       P_k_1_k_1(1,1) = 1;
       P_k_1_k_1(2,2) = 1;
       P_k_1_k_1(3,3) = 1;
       P_k_1_k_1(4,4) = 1;
       P_k_1_k_1(5,5) = 0;
       P_k_1_k_1(6,6) = 0;
       P_k_1_k_1(7,7) = 0;
   end
       
   total_diff_c = corner_1_x(n) - corner_1_x(n-1) + corner_1_y(n) - corner_1_y(n-1) ...
    + corner_2_x(n) - corner_2_x(n-1) + corner_2_y(n) - corner_2_y(n-1) ...
    + corner_3_x(n) - corner_3_x(n-1) + corner_3_y(n) - corner_3_y(n-1) ...
    + corner_4_x(n) - corner_4_x(n-1) + corner_4_y(n) - corner_4_y(n-1);

  selected_pose_ls = [0 0 0]';
       
    if((y_pos_hist(n)-local_y(n)) == 0 && (y_pos_hist(n-1)-local_y(n-1)) ~= 0)
        total_diff_s = 1;
    else
        total_diff_s = 0;
    end
  
    if(psi(n) > 1.6 || psi(n) < -1.6)
        if(psi(n) < 0)
            local_psi = psi(n)+3.14;
        else
            local_psi = psi(n)-3.14;
        end
    else
        local_psi = psi(n);
    end
    
    local_psi = 0;

    if(abs(total_diff_s) > 0)
        detect_count_h = detect_count_h + 1;
        f_fisheye = 168;
        gate_angle_offset = deg2rad(0);
        [corX, ~] = undistort_points_fish_eye_3a(side_1(n),y_p);
        side_angle_1(detect_count_h) = atan((corX-x_p)/f_fisheye)+local_psi;%
        side_ref_1(detect_count_h) = atan(-(0.5+pos_y(n))/(-pos_x(n)));%
        [corX, ~] = undistort_points_fish_eye_3a(side_2(n),y_p);
        side_angle_2(detect_count_h) = atan((corX-x_p)/f_fisheye)+local_psi;%
        side_ref_2(detect_count_h) = atan((0.5-pos_y(n))/(-pos_x(n)));%
        b = tan(side_angle_1(detect_count_h))/(tan(side_angle_2(detect_count_h))-tan(side_angle_1(detect_count_h)));
        gate_d = x_pos_hist(n);
        
        x_histogram(detect_count_h) = gate_pos(1)-gate_d;
        y_histogram(detect_count_h) = y_pos_hist(n);
        
        time_stamp_h(detect_count_h) = time_stamp(n);
        
    end

    if (corner_1_x(n) ~= 0 && abs(total_diff_c) > 1)
        %Gate position
        %#########################################################################################
        detect_count = detect_count + 1;
        prev_x_1 = corner_1_x(n);
        prev_y_1 = corner_1_y(n);
        prev_x_2 = corner_2_x(n);
        prev_y_2 = corner_2_y(n);
        prev_x_3 = corner_3_x(n);
        prev_y_3 = corner_3_y(n);
        prev_x_4 = corner_4_x(n);
        prev_y_4 = corner_4_y(n);
        
        if(1)
            image_points_x = [corner_1_x(n) corner_2_x(n) corner_3_x(n) corner_4_x(n)];
            image_points_y = [corner_1_y(n) corner_2_y(n) corner_3_y(n) corner_4_y(n)];

            [corX, corY] = undistort_points_fish_eye_2(image_points_x,image_points_y);
            
            position = [pos_x(n) pos_y(n) pos_z(n)];

            [selected_pose_ls] = pos_from_att_ned_20(gate_points,cam_pos,corX-x_p,corY-y_p,-phi(n), theta(n), 0);
            
              pos_x_ls(detect_count) = selected_pose_ls(1);%
              pos_y_ls(detect_count) = selected_pose_ls(2);%
              pos_z_ls(detect_count) = selected_pose_ls(3);

            LEAST_SQUERES_ERROR = [LEAST_SQUERES_ERROR; [pos_x_ls(n),pos_y_ls(n),pos_z_ls(n)]-[pos_x(n) pos_y(n) pos_z(n)]];
        end
        
    end

      min_dist = 0.4;
      max_dist = 1.8;
      
      sd_dist = gate_d;%
      
    if(abs(local_x(n)-local_x(n-1)) > 0  && in_turn(n) == 0)

             detect_count_m = detect_count_m + 1;
             time_stamp_m(detect_count_m) = time_stamp(n);
             pos_vision_x(detect_count_m) = local_x(n);
             pos_vision_y(detect_count_m) = local_y(n);
             pos_vision_z(detect_count_m) = pos_z(n);
    

        EKF_dt = time_stamp(n)- prev_EKF_time;
        if(EKF_dt > 1.5)
            EKF_dt = 0.02;
        end
        prev_EKF_time = time_stamp(n);

        if(f == 3 && ss_state == 0)
            ss_U_k = U_k;
            ss_P_k_1_k_1_start = P_k_1_k_1;
            ss_x_kk_1_start = x_kk_1;
            ss_dt = EKF_dt;
            ss_start = time_stamp(n);
            ss_state = 1;
        end

        if(0)%
           X_int(n,1) = arc_pred(n,1);
           X_int(n,2) = arc_pred(n,2);
        end
        
        last_vision_update_t = time_stamp(n);
        
        x_kk_1 = X_int(n,:);

        G = eye(7);
        DFx = d_kf_calc_Fx_nl(Jac_mat_f, x_kk_1, U_k);
        sum_norm = sum_norm + norm(DFx^2);
        [Phi, Gamma] = c2d(DFx, G,EKF_dt);
        Phi = c2d_A(DFx,EKF_dt);
        
        % P(k+1|k) (prediction covariance matrix) with additive noise model
        P_k_1 = Phi*P_k_1_k_1*Phi' + Q;

        DHx = d_kf_calc_Hx_nl(Jac_mat_h, x_kk_1, U_k);

        K = P_k_1 * DHx' / (DHx*P_k_1 * DHx' + R_k);

        z_k = [pos_vision_x(detect_count_m) pos_vision_y(detect_count_m) pos_z(n)];% -sonar_alt(n)];
               
        X_opt = x_kk_1' + K * (z_k - X_int(n,1:3))';
        X_int(n,:) = X_opt';

        P_k_1_k_1 = (eye(7) - K*DHx) * P_k_1 * (eye(7) - K*DHx)' + K*R_k*K';

        if(ss_state == 1)
            ss_P_k_1_k_1_end = P_k_1_k_1;
            ss_x_kk_1_end = x_kk_1;
            ss_state = 2;
        end

    end    
    
     X_int_prev = X_int(n,:);

     gate_angle = gate_initial_heading(gate_counter(n+100)+1);
     global_x = local_frame_pos_x(gate_counter(n+100)+1);
     global_y = local_frame_pos_y(gate_counter(n+100)+1);
     
       if(in_turn(n) ~= 0)
           if(in_turn(n) == -1)
               %right turn
               global_pos(n,1) = start_x + cos(psi_start+deg2rad(90))*turn_radius + sin(psi_1(n))*turn_radius;%pos x
               global_pos(n,2) = start_y + sin(psi_start+deg2rad(90))*turn_radius + cos(-psi_1(n)-deg2rad(180))*turn_radius;%pos y
               global_pos(n,3) = global_pos(n-1,3);
           else
               %left turn
               global_pos(n,1) = start_x + cos(psi_start-deg2rad(90))*turn_radius + sin(-psi_1(n))*turn_radius;%pos x
               global_pos(n,2) = start_y + sin(psi_start-deg2rad(90))*turn_radius + cos(-psi_1(n))*turn_radius;%pos y
               global_pos(n,3) = global_pos(n-1,3);
           end
       else
         rot_2d = [cos(gate_angle) -sin(gate_angle);
                  sin(gate_angle) cos(gate_angle)];

         %MATLAB EKF
         %global_pos(n,:) = [([global_x ; global_y]+rot_2d*[X_int(n,1) X_int(n,2)]')' sonar_alt(n)];% -X_int(n,3)];

         %ON_BOARD EKF
         global_pos(n,:) = [([global_x ; global_y]+rot_2d*[kf_pos_x(n) kf_pos_y(n)]')' sonar_alt(n)];% -X_int(n,3)];
         
       end
       
       EKF_m_dt(n) = EKF_dt;
    
end


Flight_pos{f} = global_pos;

end


hold on
plot3(pos_x_ls,pos_y_ls,pos_z_ls,'+')

detect_count

figure()


plot_all = 1;
if(plot_all)
    start_plotting = 2;
    [end_ , ~]= size(Flight_pos{1});
    time_offset = 0;
    plot(Flight_time{1}(start_plotting:end_)+time_offset,Flight_pos{1}(start_plotting:end_,1))
    hold on
    
    start_plotting = 2;
    [end_ , ~]= size(Flight_pos{2});
    time_offset = 0;
    plot(Flight_time{2}(start_plotting:end_)+time_offset,Flight_pos{2}(start_plotting:end_,1))
    hold on
    
    start_plotting = 2;
    [end_ , ~]= size(Flight_pos{3});
    time_offset = 0;
    plot(Flight_time{3}(start_plotting:end_)+time_offset,Flight_pos{3}(start_plotting:end_,1))
    hold on
else
    start_plotting = 2;
    [end_ , ~]= size(global_pos);
    plot(time_stamp(start_plotting:end_),global_pos(start_plotting:end_,1))
    hold on
    legend('X position estimate')
end

hold on

title('X position')
xlabel('Time [sec]') % x-axis label
ylabel('X position [M]') % y-axis label
x0=10;
y0=10;
width=1000;
height=400;
set(gcf,'units','points','position',[x0,y0,width,height])

figure()

plot_all = 1;
if(plot_all)
    start_plotting = 2;
    [end_ , ~]= size(Flight_pos{1});
    time_offset = 0;
    plot(Flight_time{1}(start_plotting:end_)+time_offset,Flight_pos{1}(start_plotting:end_,2))
    hold on
    
    start_plotting = 2;
    [end_ , ~]= size(Flight_pos{2});
   
    time_offset = 0.0;
    plot(Flight_time{2}(start_plotting:end_)+time_offset,Flight_pos{2}(start_plotting:end_,2))
    hold on
   
    start_plotting = 2;
    [end_ , ~]= size(Flight_pos{3});
    
    time_offset = 0;
    plot(Flight_time{3}(start_plotting:end_)+time_offset,Flight_pos{3}(start_plotting:end_,2))
    hold on
    legend('Flight 1','Flight 2','Flight 3','Location','northeast')
else
    start_plotting = 2;
    [end_ , ~]= size(global_pos);
    
    plot(time_stamp(start_plotting:end_),global_pos(start_plotting:end_,2))
    hold on
    legend('X position estimate')
end

title('Y position')
xlabel('Time [sec]') % x-axis label
ylabel('Y position [M]') % y-axis label

x0=10;
y0=10;
width=1000;
height=400;
set(gcf,'units','points','position',[x0,y0,width,height])

figure()

plot_all = 1;
if(plot_all)
    
    start_plotting = 2;
    [end_ , ~]= size(Flight_pos{1});
    plot(Flight_time{1}(start_plotting:end_),Flight_pos{1}(start_plotting:end_,3))
    hold on
    
    start_plotting = 2;
    [end_ , ~]= size(Flight_pos{2});
    plot(Flight_time{2}(start_plotting:end_),Flight_pos{2}(start_plotting:end_,3))
    hold on
    
    start_plotting = 2;
    [end_ , ~]= size(Flight_pos{3});
    plot(Flight_time{3}(start_plotting:end_),Flight_pos{3}(start_plotting:end_,3))
    hold on
    legend('Flight 1','Flight 2','Flight 3','Location','northeast')
else
    start_plotting = 2;
    [end_ , ~]= size(global_pos);
    plot(time_stamp(start_plotting:end_),global_pos(start_plotting:end_,3))
    hold on
    legend('Z position estimate')
end

hold on

title('Z position')
%legend('Z position ground truth','Z position estimate','Sonar measurements')
xlabel('Time [sec]') % x-axis label
ylabel('Z position [M]') % y-axis label
axis([25 40 1.1 1.7])
x0=10;
y0=10;
width=1000;
height=400;
set(gcf,'units','points','position',[x0,y0,width,height])

figure()
plot(time_stamp,kf_vel_y)
hold on
plot(time_stamp_filter,X_int(:,4))
axis([0 60 -1.0 1.0])
legend('on-board','off-board')
title('Z speed')

% bias 
figure
plot(time_stamp_filter,X_int(:,5))
hold on
plot(time_stamp,acc_bias_x)
legend('off-board','on-board')
title('Accelerometer bias X')
xlabel('Time in sec') % x-axis label
ylabel('X bias in m/s2') % y-axis label
axis([0 40 0.0 0.07])
x0=10;
y0=10;
width=1000;
height=200;
set(gcf,'units','points','position',[x0,y0,width,height])

%bias 
figure
plot(time_stamp_filter,X_int(:,6))
hold on
plot(time_stamp,acc_bias_y)
legend('off-board','on-board')
title('Accelerometer bias Y')
xlabel('Time in sec') % x-axis label
ylabel('Y bias in m/s2') % y-axis label
axis([0 40 -0.1 0.0])
x0=10;
y0=10;
width=1000;
height=200;
set(gcf,'units','points','position',[x0,y0,width,height])

figure
plot(time_stamp_filter,X_int(:,7))
hold on
plot(time_stamp,acc_bias_z)
legend('off-board','on-board')
title('Accelerometer bias Z')
xlabel('Time in sec') % x-axis label
ylabel('Z bias in m/s2') % y-axis label
axis([0 95 0 0.5])
x0=10;
y0=10;
width=1000;
height=200;
set(gcf,'units','points','position',[x0,y0,width,height])


%3D plotting
gate_x = [6,7,2.25,-1.5,-3];
gate_y = [0,-11,-8,-8.5,-2];
gate_psi = [0,-90.0,-270.0,-150.0,-270.0];

figure()
g_2_x = 0;
g_2_y = 3;
test_3d_plot_2 = [0+g_2_x 0+g_2_y 0; 0+g_2_x 0+g_2_y 1;0+g_2_x -0.5+g_2_y 1;0+g_2_x -0.5+g_2_y 2;0+g_2_x 0.5+g_2_y 2;0+g_2_x 0.5+g_2_y 1;0+g_2_x 0+g_2_y 1];

start_plotting = 2000;
[end_ , ~]= size(X_int);
end_ = data_size;%

plot_all = 1;
if(plot_all)
    start_plotting = 2;
    [end_ , ~]= size(Flight_pos{1});
    end_ = 17000;
    plot3(Flight_pos{1}(start_plotting:end_,1),-Flight_pos{1}(start_plotting:end_,2),Flight_pos{1}(start_plotting:end_,3))
    hold on
    
    start_plotting = 2;
    [end_ , ~]= size(Flight_pos{2});
    end_ = 18250;
    plot3(Flight_pos{2}(start_plotting:end_,1),-Flight_pos{2}(start_plotting:end_,2),Flight_pos{2}(start_plotting:end_,3))
    hold on
    
    start_plotting = 2;
    [end_ , ~]= size(Flight_pos{3});
    end_ = 15600;
    plot3(Flight_pos{3}(start_plotting:end_,1),-Flight_pos{3}(start_plotting:end_,2),Flight_pos{3}(start_plotting:end_,3))
    hold on

else
    start_plotting = 2;
    [end_ , ~]= size(global_pos);
    plot3(global_pos(start_plotting:end_,1),-global_pos(start_plotting:end_,2),global_pos(start_plotting:end_,3))
    hold on
end

for i = 1:5
%gate at origin
g_1_x = 0.0;
g_1_y = 0.0;
test_3d_plot = [0+g_1_x 0+g_1_y 0; 0+g_1_x 0+g_1_y 1;0+g_1_x -0.5+g_1_y 1;0+g_1_x -0.5+g_1_y 2;0+g_1_x 0.5+g_1_y 2;0+g_1_x 0.5+g_1_y 1;0+g_1_x 0+g_1_y 1];
gate_angle = deg2rad(gate_psi(i));
rot_2d = [cos(gate_angle) -sin(gate_angle);
                  sin(gate_angle) cos(gate_angle)];
temp = rot_2d*test_3d_plot(1:7,1:2)';
test_3d_plot(1:7,1:2) = temp';
plot3(test_3d_plot(:,1)+gate_x(i),-(test_3d_plot(:,2)+gate_y(i)),test_3d_plot(:,3),'Linewidth',3,'Color','r')
hold on
end

axis([-7 9, -5 14, 0 3.5])
legend('Flight 1','Flight 2','Flight 3','Location','northeast')
xlabel('X position [M]') % x-axis label
ylabel('Y position [M]') % y-axis label
zlabel('Z position [M]') % z-axis label
title('3D Position estimate')
view([-54,78])
grid on

saveas(gcf,'../autosave/png/Basement_flights_3d_position.png')
saveas(gcf,'../autosave/eps/Basement_flights_3d_position','epsc')

%2D plotting
gate_x = [6,7,2.25,-1.5,-3.3];
gate_y = [0,-11,-8,-8.5,-2];
gate_psi = [0,-90.0,-270.0,-150.0,-270.0];

figure()

plot_all = 1;
if(plot_all)
    start_plotting = 2;
    [end_ , ~]= size(Flight_pos{1});
    end_ = 17000;
    plot(Flight_pos{1}(start_plotting:end_,1),-Flight_pos{1}(start_plotting:end_,2))
    hold on
    
    start_plotting = 2;
    [end_ , ~]= size(Flight_pos{2});
    end_ = 18250;
    plot(Flight_pos{2}(start_plotting:end_,1),-Flight_pos{2}(start_plotting:end_,2))
    hold on
    
    start_plotting = 2;
    [end_ , ~]= size(Flight_pos{3});
    end_ = 15600;
    plot(Flight_pos{3}(start_plotting:end_,1),-Flight_pos{3}(start_plotting:end_,2))
    hold on

else
    start_plotting = 2;
    [end_ , ~]= size(global_pos);
    plot(global_pos(start_plotting:end_,1),-global_pos(start_plotting:end_,2))
    hold on
end

for i = 1:5
%gate at origin
g_1_x = 0.0;
g_1_y = 0.0;
test_3d_plot = [0+g_1_x 0+g_1_y 0; 0+g_1_x 0+g_1_y 1;0+g_1_x -0.5+g_1_y 1;0+g_1_x -0.5+g_1_y 2;0+g_1_x 0.5+g_1_y 2;0+g_1_x 0.5+g_1_y 1;0+g_1_x 0+g_1_y 1];
gate_angle = deg2rad(gate_psi(i));
rot_2d = [cos(gate_angle) -sin(gate_angle);
                  sin(gate_angle) cos(gate_angle)];
temp = rot_2d*test_3d_plot(1:7,1:2)';
test_3d_plot(1:7,1:2) = temp';
plot(test_3d_plot(:,1)+gate_x(i),-(test_3d_plot(:,2)+gate_y(i)),'Linewidth',3,'Color','r')
hold on
end

axis([-7 9, -3 18])
legend('Flight 1','Flight 2','Flight 3','Location','northeast')
xlabel('X position [M]') % x-axis label
ylabel('Y position [M]') % y-axis label
title('Position estimate')
grid on
x = [0.4 0.5];
y = [0.2 0.2];
a = annotation('textarrow',x,y,'String','Start ')
a.FontSize = 14;

saveas(gcf,'../autosave/png/Basement_flights_2d_position.png')
saveas(gcf,'../autosave/eps/Basement_flights_2d_position','epsc')


K1 = -0.54;%
K2 = -0.9;% 

fc = 20; % Cut off frequency
fs = 512; % Sampling rate

[b,a] = butter(6,fc/(fs/2)); % Butterworth filter of order 6
acc_x_filter = filter(b,a,acc_x); 
acc_y_filter = filter(b,a,acc_y); 
acc_z_filter = filter(b,a,acc_z); 

gyro_p_filter = filter(b,a,gyro_p); 
gyro_p_diff = diff(gyro_p_filter);

fc = 2; % Cut off frequency
fs = 512; % Sampling rate
[b,a] = butter(6,fc/(fs/2)); % Butterworth filter of order 6
v_x_filter_mov = movmean(((acc_x_filter-X_int(:,5))/K1),512);%
v_x_filter = filter(b,a,((acc_x_filter-X_int(:,5))/K1)); 
figure
plot(time_stamp,acc_x_filter/K1)
hold on
plot(time_stamp,(acc_x_filter-X_int(:,5))/K1)
hold on
plot(time_stamp,body_speed(:,1))
hold on
plot(time_stamp,v_x_filter)
hold on
plot(time_stamp,v_x_filter_mov)
legend('X body v','X body v bias','X body v optitrack','v_x_filter','v_x_filter_mov')
title('x body velocity prediction')

figure
v_x_diff = body_speed(:,1)-v_x_filter;
v_x_mov_diff = body_speed(:,1)-v_x_filter_mov;
v_x_mov_diff_shift = body_speed(:,1)-circshift(v_x_filter_mov,-250);
plot(v_x_diff)
hold on
plot(v_x_mov_diff)
hold on
plot(v_x_mov_diff_shift)
legend('v_x_diff','v_x_mov_diff','v_x_mov_diff_shift')
title('x body velocity error')

figure
plot(time_stamp,acc_y_filter/K2)
hold on
plot(time_stamp,(acc_y_filter-X_int(:,6))/K2)
hold on
plot(time_stamp,body_speed(:,2))
title('y body velocity prediction')

figure
plot(psi_1)


figure
plot(time_stamp_m,pos_vision_x,'+')
hold on
plot(time_stamp,X_int(:,1))
hold on 
plot(time_stamp,kf_pos_x)
legend('pos_vision_x','off-board','on-board')
title('X EKF estimates');

figure
plot(time_stamp_m,pos_vision_y,'+')
hold on
plot(time_stamp,X_int(:,2))
hold on
plot(time_stamp,kf_pos_y)
legend('pos_vision_y','off-board','on-board')
title('Y EKF estimates');

figure
plot(time_stamp,X_int(:,3))
hold on 
plot(time_stamp,kf_pos_z)
hold on
plot(time_stamp,pos_z)
legend('off-board','on-board','pprz altitude')
title('Z EKF estimates');

figure()
plot(time_stamp,EKF_m_dt)
title('EKF_m_dt')

%Snapshot
ss_U_k
ss_x_kk_1_start
ss_x_kk_1_end
ss_P_k_1_k_1_start
ss_P_k_1_k_1_end

ss_dt


