close all;
clear all;

%gate position at x = 4.2 y = 0.8 z = -1.4 (ned)
%uncomment the desired dataset for plotting

VarName = csvread('filter_debug_lsd_2.csv');

counter = VarName(:,1); 
%time_stamp = VarName(:,2);

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
sonar_alt = VarName(:,44)+0.1;%;

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

side_1 = VarName(:,55);
side_2 = VarName(:,56);


[data_size dummy ] = size(counter);

data_size

time_stamp = (0:1/512:(data_size/512)-(1/512))';

prev_sonar = 0;
%sonar spike removal
block = 0;
start_block = 0;
block_time = 0;
sonar_filter = zeros(data_size,1);
prev_diff = 0;
prev_diff_time = 0;
for n = 2:data_size
   
   if(block == 0 &&(sonar_alt(n)-sonar_alt(n-1))>0.1)
       block = 1;
       start_block = sonar_alt(n-1);
       block_time = time_stamp(n);
   elseif(block == 0 &&(sonar_alt(n)-sonar_alt(n-1))<-0.1)
       block = -1;
       start_block = sonar_alt(n-1);
       block_time = time_stamp(n);
   end
   
   if(block > 0 && ( sonar_alt(n)-sonar_filter(n-1) < 0.1 || time_stamp(n) - block_time > 0.1))
       block = 0;
   elseif(block < 0 && ( sonar_alt(n)-sonar_filter(n-1) > -0.1 || time_stamp(n) - block_time > 0.5))
       block = 0;
   end
   
   if(block ~= 0)
       sonar_filter(n) = start_block;
   else
       sonar_filter(n) = sonar_alt(n);
   end
   
   if(abs(sonar_filter(n)-sonar_filter(n-1))>0)
       prev_diff = sonar_filter(n-1);
       prev_diff_time = time_stamp(n);
   end
   
   if(time_stamp(n)-prev_diff_time > 0.05)
       sonar_filter(n-2) = prev_diff;
   end
   
end



 sonar_filter =  movmean(sonar_filter,[75 0]);

detect_count = 0;

prev_x_1 = 0;
prev_y_1 = 0;
prev_x_2 = 0;
prev_y_2 = 0;
prev_x_3 = 0;
prev_y_3 = 0;
prev_x_4 = 0;
prev_y_4 = 0;

gate_pos = [3.5 0.05 -1.4];
cam_pos = [0 0 0];%unknown dummy
[ gate_points ] = Calc_gate_points_order( gate_pos );

ls_position = [0 0 0];
angle_position = [0 0 0];
LEAST_SQUERES_ERROR = [0 0 0];

%EKF 

pos_x_ls = zeros(data_size,1);
pos_y_ls = zeros(data_size,1);
pos_z_ls = zeros(data_size,1);

psi_offset = deg2rad(0);%
%initialization
prev_time  = time_stamp(1);
 pos_nav = [0 0 0];
 pos_hold = [0 0 0];
 pos_out = [0 0 0];
 pos_out_time = 0;
prev_pos = pos_nav;

%calibration pqr and acc
end_init = 300;

gyro_bias = [ mean(gyro_p(1:end_init)) mean(gyro_q(1:end_init)) mean(gyro_r(1:end_init))];

%EKF init
EKF_count = 0;
HOLD_count = 0;
prev_EKF_time = 0;
EKF_time_vec = [0];
count = 0;

init_period = 5.5;%

bias_acc  = [0 0 0]';
bias_gyro = [0 0 0]';%

X_int = [0 0 0 0 0 0 0];%
X_dot = [0 0 0 0 0 0 0];
X_int_prev = X_int;

P_k_1 = zeros(7,7);
P_k_1_k_1 = eye(7,7)*1;%initialize with large, or original variances?

Q = diag([0.2 0.2 0.1 4.0 0.0 0.0 0.0]);%2.0

R_k = diag([0.10 0.10 0.1]);

% extract position from state prediction
H_k = [eye(3,3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3)];
DHx = eye(3);

%variables at ekf rate
acc_x_raw = [0];

Jac_mat_f = d_kf_calc_jacobian();%
Jac_mat_h = d_kf_calc_jacobian_h();

%turn logic
in_turn = zeros(data_size,1);
start_x = 0;
start_y = 0;
start_z = 0;

arc_pred = zeros(data_size,6);

%stop = data_size
stop = 30000;%

time_stamp_filter = time_stamp(1:data_size);%now X_int is prealocated so filter time stamp can be the same as normal time stamp

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
detect_count_a = 0;

gate_d = 0;
gate_close = 0;

%test expm approximation
sum_norm = 0;

body_speed = zeros(data_size,3); 

X_int = zeros(data_size,7);

last_vision_update_t = 0;

%load vision heading estimates which are processed in corner_processing_vision_heading.m  
load('filter_psi_lsd_2.mat')
psi_mod = mod(filter_psi',2*pi);
psi_filter_heading = zeros(data_size,1);
for i = 1:data_size
    if(psi_mod(i)>pi)
        psi_filter_heading(i) = psi_mod(i)-2*pi;
    else
        psi_filter_heading(i) = psi_mod(i);
    end
end
psi=psi_filter_heading;

for n = 2:stop
    
   dt = time_stamp(n)-time_stamp(n-1);
   omega = [gyro_p(n) gyro_q(n)];
   
   body_speed(n,:) = (C_b_n(phi(n),theta(n),psi(n))'*[vel_x(n) vel_y(n) vel_z(n)]')';
 
   U_k = [acc_x_filter(n) acc_y_filter(n) acc_z_filter(n) omega phi(n) theta(n) psi(n)];
   
   dot = d_kf_calc_f_nl(dt,X_int_prev,U_k);
   X_dot = [X_dot; dot'];

   X_int(n,:) = X_int_prev + dot'*dt;
   
   pos_nav = [pos_nav;X_int(1:3)];
   
   %turning: assuming no lateral and vertical speed when entering turn
   
   if(in_turn(n-1) == 0 && phi(n)>deg2rad(8))
       in_turn(n) = 1;
       
       start_x = X_int(n,1);%
       start_y = X_int(n,2);%
       start_z = X_int(n,3);%
       
   elseif(in_turn(n-1) == 1 && phi(n)<deg2rad(8))
       in_turn(n) = 0;
   else
       in_turn(n) = in_turn(n-1);
   end
   
   if(in_turn(n) == 1)
       if(pos_x(n)>2.5)
           %first turn
           arc_pred(n,1) = start_x + sin(psi(n))*1.5;%pos x
           arc_pred(n,2) = start_y + 1.5 -cos(psi(n))*1.5;%pos y
       else
           %second turn
           arc_pred(n,1) = start_x + sin(psi(n))*1.5;%pos x
           arc_pred(n,2) = start_y - 1.5 - cos(psi(n))*1.5;%pos y
       end
   else
       
   end
   
   %turn ended reset position and speed covariance
   if((in_turn(n)-in_turn(n-1)) < 0)
       P_k_1_k_1(1,1) = 1;
       P_k_1_k_1(2,2) = 1;
       P_k_1_k_1(3,3) = 1;
        P_k_1_k_1(4,4) = 0.5;
   end
       
   total_diff_c = corner_1_x(n) - corner_1_x(n-1) + corner_1_y(n) - corner_1_y(n-1) ...
    + corner_2_x(n) - corner_2_x(n-1) + corner_2_y(n) - corner_2_y(n-1) ...
    + corner_3_x(n) - corner_3_x(n-1) + corner_3_y(n) - corner_3_y(n-1) ...
    + corner_4_x(n) - corner_4_x(n-1) + corner_4_y(n) - corner_4_y(n-1);

  selected_pose_ls = [0 0 0]';
  
  total_diff_s = side_1(n) - side_1(n-1) + side_2(n) - side_2(n-1);
  
    if(psi(n) > 1.6 || psi(n) < -1.6)
        if(psi(n) < 0)
            local_psi = psi(n)+3.14;
        else
            local_psi = psi(n)-3.14;
        end
    else
        local_psi = psi(n);
    end

    if(abs(total_diff_s) > 0)
        detect_count_h = detect_count_h + 1;
        f_fisheye = 168;
        gate_angle_offset = deg2rad(6.5);
        [corX, ~] = undistort_points_fish_eye_3a(side_1(n),y_p);
        side_angle_1(detect_count_h) = atan((corX-x_p)/f_fisheye)+local_psi+gate_angle_offset;%
        side_ref_1(detect_count_h) = atan(-(0.5+pos_y(n))/(-pos_x(n)));%
        [corX, ~] = undistort_points_fish_eye_3a(side_2(n),y_p);
        side_angle_2(detect_count_h) = atan((corX-x_p)/f_fisheye)+local_psi+gate_angle_offset;%
        side_ref_2(detect_count_h) = atan((0.5-pos_y(n))/(-pos_x(n)));%
        b = tan(side_angle_1(detect_count_h))/(tan(side_angle_2(detect_count_h))-tan(side_angle_1(detect_count_h)));
        gate_d = (b/tan(side_angle_1(detect_count_h)));
        
         if(pos_y(n) > 1.5)
             x_histogram(detect_count_h) = gate_d-1.0;
             gate_close = X_int(n,1);%for filtering hisyogram when to close to the gate
             y_histogram(detect_count_h) = (b+0.5)+3.0;
         end
         if(pos_y(n) < 1.5)
              x_histogram(detect_count_h) = 3.9-gate_d+0.25+0.1;
              gate_close = 4.15-X_int(n,1);%for filtering histogram when to close to the gate
              y_histogram(detect_count_h) = -(b+0.5) + 0.05;
         end
        
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
        n

        opti_pos = [pos_x(n) pos_y(n) pos_z(n)];
        
        %offset of optitrack heading with track heading 
        psi_offset = deg2rad(-6.5);
        
        if(pos_y(n) > 1.5)
            image_points_x = [corner_1_x(n) corner_2_x(n) corner_3_x(n) corner_4_x(n)];
            image_points_y = [corner_1_y(n) corner_2_y(n) corner_3_y(n) corner_4_y(n)];
            
            [corX, corY] = undistort_points_fish_eye_4a(image_points_x,image_points_y,cameraParams_flow_speed);

            if(rad2deg(psi(n))<0)
                psi_2 = psi(n)+deg2rad(180);
            else
                psi_2 = psi(n)-deg2rad(180);
            end

            [selected_pose_ls] = pos_from_att_ned_20(gate_points,cam_pos,corX-x_p,corY-y_p,-phi(n), theta(n), -psi_2 + psi_offset); 

            %lsd2
            pos_x_ls(detect_count) = 3.0-selected_pose_ls(1)+0.2;
            pos_y_ls(detect_count) = -selected_pose_ls(2)+3.15;
            pos_z_ls(detect_count) = selected_pose_ls(3)-0.1;
 
            LEAST_SQUERES_ERROR = [LEAST_SQUERES_ERROR; [pos_x_ls(n),pos_y_ls(n),pos_z_ls(n)]-[pos_x(n) pos_y(n) pos_z(n)]];

        end

        if(pos_y(n) < 1.5)
            image_points_x = [corner_1_x(n) corner_2_x(n) corner_3_x(n) corner_4_x(n)];
            image_points_y = [corner_1_y(n) corner_2_y(n) corner_3_y(n) corner_4_y(n)];

            [corX, corY] = undistort_points_fish_eye_4a(image_points_x,image_points_y,cameraParams_flow_speed);

            position = [pos_x(n) pos_y(n) pos_z(n)];

            [selected_pose_ls] = pos_from_att_ned_20(gate_points,cam_pos,corX-x_p,corY-y_p,-phi(n), theta(n), -psi(n) + psi_offset);
            

            %lsd2
              pos_x_ls(detect_count) = selected_pose_ls(1)+0.1;
              pos_y_ls(detect_count) = selected_pose_ls(2);
              pos_z_ls(detect_count) = selected_pose_ls(3);
            
            LEAST_SQUERES_ERROR = [LEAST_SQUERES_ERROR; [pos_x_ls(n),pos_y_ls(n),pos_z_ls(n)]-[pos_x(n) pos_y(n) pos_z(n)]];
        end
        
    end

      min_dist = 0.4;
      max_dist = 1.8;
    
      
      sd_dist = gate_d;%
      
    if(((corner_1_x(n) ~= 0 &&abs(total_diff_c) > 1) || (sd_dist > min_dist && sd_dist < max_dist && abs(total_diff_s) > 0 ))  && in_turn(n) == 0)
        
        if(sd_dist > min_dist && sd_dist < max_dist && abs(total_diff_s) > 0)
            detect_count_m = detect_count_m + 1;
            time_stamp_m(detect_count_m) = time_stamp(n);
            pos_vision_x(detect_count_m) = x_histogram(detect_count_h);
            pos_vision_y(detect_count_m) = y_histogram(detect_count_h);
            pos_vision_z(detect_count_m) = pos_z(n);
        elseif(corner_1_x(n) ~= 0 && abs(total_diff_c) > 1)
            detect_count_m = detect_count_m + 1;
            time_stamp_m(detect_count_m) = time_stamp(n);
            pos_vision_x(detect_count_m) = pos_x_ls(detect_count);
            pos_vision_y(detect_count_m) = pos_y_ls(detect_count);
            pos_vision_z(detect_count_m) = pos_z_ls(detect_count);
        end

        EKF_dt = time_stamp(n)- prev_EKF_time;
        prev_EKF_time = time_stamp(n);
        
        last_vision_update_t = time_stamp(n);
        
        x_kk_1 = X_int(n,:);
        
        G = eye(7);
        DFx = d_kf_calc_Fx_nl(Jac_mat_f, x_kk_1, U_k);
        sum_norm = sum_norm + norm(DFx^2);
        [Phi, Gamma] = c2d(DFx, G,EKF_dt);
        Phi = c2d_A(DFx,EKF_dt);
        
        % P(k+1|k) (prediction covariance matrix) with additive noise model
        P_k_1 = Phi*P_k_1_k_1*Phi' + Q;%

        DHx = d_kf_calc_Hx_nl(Jac_mat_h, x_kk_1, U_k);

        K = P_k_1 * DHx' / (DHx*P_k_1 * DHx' + R_k);

        z_k = [pos_vision_x(detect_count_m) pos_vision_y(detect_count_m) -sonar_filter(n) ];%-sonar_alt(n)];
       
        X_opt = x_kk_1' + K * (z_k - X_int(n,1:3))';
        X_int(n,:) = X_opt';

        P_k_1_k_1 = (eye(7) - K*DHx) * P_k_1 * (eye(7) - K*DHx)' + K*R_k*K';

              %update using arc prediction if new sonar measurmement
    elseif(in_turn(n) == 1 && abs(sonar_alt(n) - sonar_alt(n-1)) > 0 )
        
         EKF_dt = time_stamp(n)- prev_EKF_time;
        prev_EKF_time = time_stamp(n);
        
        x_kk_1 = X_int(n,:);
       
        G = eye(7);
        DFx = d_kf_calc_Fx_nl(Jac_mat_f, x_kk_1, U_k);
        sum_norm = sum_norm + norm(DFx^2);
        [Phi, Gamma] = c2d(DFx, G,EKF_dt);
        Phi = c2d_A(DFx,EKF_dt);

        % P(k+1|k) (prediction covariance matrix) with additive noise
        P_k_1 = Phi*P_k_1_k_1*Phi' + Q;%

        DHx = d_kf_calc_Hx_nl(Jac_mat_h, x_kk_1, U_k);

        K = P_k_1 * DHx' / (DHx*P_k_1 * DHx' + R_k);

        z_k = [arc_pred(n,1) arc_pred(n,2) -sonar_filter(n) ];%
       
        X_opt = x_kk_1' + K * (z_k - X_int(n,1:3))';
        X_int(n,:) = X_opt';

        P_k_1_k_1 = (eye(7) - K*DHx) * P_k_1 * (eye(7) - K*DHx)' + K*R_k*K';
        
    elseif(1&&time_stamp(n) > 25 && abs(sonar_filter(n) - sonar_filter(n-1)) > 0 && time_stamp(n) - prev_EKF_time > 0.16)
 
         EKF_dt = time_stamp(n)- prev_EKF_time;
        prev_EKF_time = time_stamp(n);
        
        x_kk_1 = X_int(n,:);
 
        G = eye(7);
        DFx = d_kf_calc_Fx_nl(Jac_mat_f, x_kk_1, U_k);
        sum_norm = sum_norm + norm(DFx^2);
        [Phi, Gamma] = c2d(DFx, G,EKF_dt);
        Phi = c2d_A(DFx,EKF_dt);

        % P(k+1|k) (prediction covariance matrix) with additive noise
        P_k_1 = Phi*P_k_1_k_1*Phi' + Q;%

       DHx = [ 0     0     0     0     0     0     0;
               0     0     0     0     0     0     0;
               0     0     1     0     0     0     0];
        

        K = P_k_1 * DHx' / (DHx*P_k_1 * DHx' + R_k);

        z_k = [arc_pred(n,1) arc_pred(n,2) -sonar_filter(n)];
       
        X_opt = x_kk_1' + K * (z_k - X_int(n,1:3))';
        X_int(n,:) = X_opt';

        P_k_1_k_1 = (eye(7) - K*DHx) * P_k_1 * (eye(7) - K*DHx)' + K*R_k*K';

    end    
    
     X_int_prev = X_int(n,:);
    
     ls_position = [ls_position;selected_pose_ls'];
end

hold on
plot3(pos_x_ls,pos_y_ls,pos_z_ls,'+')

detect_count

figure()
plot(time_stamp,pos_x)
hold on
plot(time_stamp_filter,X_int(:,1))
hold on
Arc_pred_1 = arc_pred(:,1);
plot(time_stamp,Arc_pred_1,time_stamp_m,pos_vision_x,'+')
legend('X position Optitrack','X position estimate','Arc prediction','Vision measurements')
title('X position')
xlabel('Time [sec]') % x-axis label
ylabel('X position [M]') % y-axis label
axis([25 40 -2 6])
x0=10;
y0=10;
width=500;
height=200;
set(gcf,'units','points','position',[x0,y0,width,height])
saveas(gcf,'../autosave/png/EKF_pos_X_4.png')
saveas(gcf,'../autosave/eps/EKF_pos_X_4','epsc')

figure()
plot(time_stamp,pos_y)
hold on
plot(time_stamp_filter,X_int(:,2))
hold on
plot(time_stamp,arc_pred(:,2),time_stamp_m,pos_vision_y,'+')
legend('Y position Optitrack','Y position estimate','Arc prediction','Vision measurements')
title('Y position')
xlabel('Time [sec]') % x-axis label
ylabel('Y position [M]') % y-axis label
axis([25 40 -0.5 3.5])
x0=10;
y0=10;
width=500;
height=200;
set(gcf,'units','points','position',[x0,y0,width,height])
saveas(gcf,'../autosave/png/EKF_pos_Y_4.png')
saveas(gcf,'../autosave/eps/EKF_pos_Y_4','epsc')

figure()
plot(time_stamp,-pos_z)
hold on
plot(time_stamp_filter,-X_int(:,3))
hold on
plot(time_stamp,sonar_filter)
title('Z position')
legend('Z position Optitrack','Z position estimate','Sonar height')
xlabel('Time [sec]') % x-axis label
ylabel('Z position [M]') % y-axis label
axis([25 40 1.2 1.8])
x0=10;
y0=10;
width=500;
height=200;
set(gcf,'units','points','position',[x0,y0,width,height])
saveas(gcf,'../autosave/png/EKF_pos_Z_4.png')
saveas(gcf,'../autosave/eps/EKF_pos_Z_4','epsc')

figure
pos_error_x = X_int(:,1)-pos_x;
histogram(pos_error_x(14000:28000),'EdgeAlpha',0.0,'FaceColor','k')
title('X error distribution')
xlabel('Error [M]') % x-axis label
ylabel('Frequency') % y-axis label
axis([-inf inf 0 2000])
x0=10;
y0=10;
width=200;
height=200;
set(gcf,'units','points','position',[x0,y0,width,height])
saveas(gcf,'../autosave/png/EKF_pos_error_X.png')
saveas(gcf,'../autosave/eps/EKF_pos_error_X','epsc')

figure
pos_error_y = X_int(:,2)-pos_y;
histogram(pos_error_y(14000:28000),'EdgeAlpha',0.0,'FaceColor','k')
title('Y error distribution')
xlabel('Error [M]') % x-axis label
ylabel('Frequency') % y-axis label
axis([-inf inf 0 2000])
x0=10;
y0=10;
width=200;
height=200;
set(gcf,'units','points','position',[x0,y0,width,height])
saveas(gcf,'../autosave/png/EKF_pos_error_Y.png')
saveas(gcf,'../autosave/eps/EKF_pos_error_Y','epsc')

figure
pos_error_z = X_int(:,3)-pos_z;
histogram(pos_error_z(14000:28000),'EdgeAlpha',0.0,'FaceColor','k')
title('Z error distribution')
xlabel('Error [M]') % x-axis label
ylabel('Frequency') % y-axis label
axis([-inf inf 0 2000])
x0=10;
y0=10;
width=200;
height=200;
set(gcf,'units','points','position',[x0,y0,width,height])
saveas(gcf,'../autosave/png/EKF_pos_error_Z.png')
saveas(gcf,'../autosave/eps/EKF_pos_error_Z','epsc')

figure()
plot(time_stamp,body_speed(:,3))
hold on
plot(time_stamp_filter,X_int(:,4))
hold on
axis([25 40 -0.5 0.2])
x0=10;
y0=10;
width=500;
height=200;
set(gcf,'units','points','position',[x0,y0,width,height])
legend('Optitrack Z body velocity','Z body velocity estimate')
xlabel('Time [sec]') % x-axis label
ylabel('Z body velocity [M/s]') % y-axis label
title('Z body velocity')
saveas(gcf,'../autosave/png/EKF_vel_Z_4.png')
saveas(gcf,'../autosave/eps/EKF_vel_Z_4','epsc')

% bias 
figure
plot(time_stamp_filter,X_int(:,5))
title('Accelerometer bias X')
xlabel('Time [sec]') % x-axis label
ylabel('X bias [M/s2]') % y-axis label
axis([0 40 0.0 0.07])
x0=10;
y0=10;
width=500;
height=200;
set(gcf,'units','points','position',[x0,y0,width,height])
saveas(gcf,'../autosave/png/EKF_bias_X_4.png')
saveas(gcf,'../autosave/eps/EKF_bias_X_4','epsc')

%bias 
figure
plot(time_stamp_filter,X_int(:,6))
title('Accelerometer bias Y')
xlabel('Time [sec]') % x-axis label
ylabel('Y bias [M/s2]') % y-axis label
axis([0 40 -0.1 0.0])
x0=10;
y0=10;
width=500;
height=200;
set(gcf,'units','points','position',[x0,y0,width,height])
saveas(gcf,'../autosave/png/EKF_bias_Y_4.png')
saveas(gcf,'../autosave/eps/EKF_bias_Y_4','epsc')

figure
plot(time_stamp_filter,X_int(:,7))
title('Accelerometer bias Z')
xlabel('Time [sec]') % x-axis label
ylabel('Z bias [M/s2]') % y-axis label
axis([0 40 0 0.5])
x0=10;
y0=10;
width=500;
height=200;
set(gcf,'units','points','position',[x0,y0,width,height])
saveas(gcf,'../autosave/png/EKF_bias_Z_4.png')
saveas(gcf,'../autosave/eps/EKF_bias_Z_4','epsc')

%gate at origin
g_1_x = 3.5;
g_1_y = 0;
g_2_x = 0;
g_2_y = 3;
test_3d_plot = [0+g_1_x 0+g_1_y 0; 0+g_1_x 0+g_1_y 1;0+g_1_x -0.5+g_1_y 1;0+g_1_x -0.5+g_1_y 2;0+g_1_x 0.5+g_1_y 2;0+g_1_x 0.5+g_1_y 1;0+g_1_x 0+g_1_y 1];
test_3d_plot_2 = [0+g_2_x 0+g_2_y 0; 0+g_2_x 0+g_2_y 1;0+g_2_x -0.5+g_2_y 1;0+g_2_x -0.5+g_2_y 2;0+g_2_x 0.5+g_2_y 2;0+g_2_x 0.5+g_2_y 1;0+g_2_x 0+g_2_y 1];

start_plotting = 2000;
[end_ , ~]= size(X_int);
end_ = 27500;
figure()
plot3(pos_x(start_plotting:end_),pos_y(start_plotting:end_),-pos_z(start_plotting:end_))
grid on
hold on
plot3(X_int(start_plotting:end_,1),X_int(start_plotting:end_,2),-X_int(start_plotting:end_,3))
hold on
plot3(test_3d_plot(:,1),test_3d_plot(:,2),test_3d_plot(:,3),'Linewidth',3,'Color','r')
hold on
plot3(test_3d_plot_2(:,1),test_3d_plot_2(:,2),test_3d_plot_2(:,3),'Linewidth',3,'Color','r')
axis([-3 6, -2 4, 0 3.5])
legend('Position Optitrack','Position estimate','Location','northeast')
xlabel('X position [M]') % x-axis label
ylabel('Y position [M]') % y-axis label
zlabel('Z position [M]') % z-axis label
title('3D position')
saveas(gcf,'../autosave/png/EKF_pos_3D_gates.png')
saveas(gcf,'../autosave/eps/EKF_pos_3D_gates','epsc')


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

end
figure
plot(psi_1)

figure
plot(psi)

