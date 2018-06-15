
run_processing = 1;

if run_processing
    clear all;
    run_processing = 1;
end

close all;


%Gate-based heading estimation with real vision data
%gate position at x = 4.2 y = 0.8 z = -1.4 (ned)
%uncomment the desired dataset for plotting


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%SESSION 8
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%VarName = csvread('Inf_loop_tests\session_8\flight_10.csv');
% opti_data = import_optitrack_data_2('Inf_loop_tests\session_6\Take 2017-12-20 07.11.41 PM_f4',{'RigidBody 4'})
% time_shift = 6.5;

VarName = csvread('Inf_loop_tests\session_8\flight_12_long.csv');
opti_data = import_optitrack_data_2('Inf_loop_tests\session_6\Take 2017-12-20 07.11.41 PM_f4',{'RigidBody 4'})
time_shift = 6.5;
%%used for article plot


gyro_heading_flight = 1

matlab_undistort = 1;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% stop = array_size(1);

%rename used variables
counter = VarName(:,1); time_stamp = VarName(:,2);

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

sonar_alt = VarName(:,44);

%spike removal
sonar_agl = medfilt1(sonar_alt,6);

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

%dummy data for log file without histogram logging(side 1,2)
side_1 = VarName(:,53);
side_2 = VarName(:,54);


[data_size dummy ] = size(counter);

data_size

figure()
plot(corner_1_x,corner_1_y,'+')
hold on
plot(corner_2_x,corner_2_y,'+')
hold on
plot(corner_3_x,corner_3_y,'+')
hold on
plot(corner_4_x,corner_4_y,'+')
axis([0 320, 0 160])

detect_count = 0;
detect_count_h = 0;
detect_count_m = 0;
detect_count_a = 0;

prev_x_1 = 0;
prev_y_1 = 0;
prev_x_2 = 0;
prev_y_2 = 0;
prev_x_3 = 0;
prev_y_3 = 0;
prev_x_4 = 0;
prev_y_4 = 0;

gate_pos = [3.5 0.05 -1.4];
gate_pos_neu = [3.5 0.05 1.4];
cam_pos = [0 0 0];%unknown dummy
[ gate_points ] = Calc_gate_points_order( gate_pos );

ls_position = [0 0 0];
angle_position = [0 0 0];
LEAST_SQUERES_ERROR = [0 0 0];

%matlab 3 coeficient radial distortion correction
%load('calib_Flow_3c.mat');
load('calib_rot_3.mat');

 if(matlab_undistort)
        x_p = cameraParams_rot_3.PrincipalPoint(1)%
        y_p = 160 - cameraParams_rot_3.PrincipalPoint(2)%
 else
        x_p = 153.2
        y_p = 32
 end


end_init = 3000;
gyro_bias = [ mean(gyro_p(1:end_init)) mean(gyro_q(1:end_init)) mean(gyro_r(1:end_init))];

OPTITRACK_OFFSET = 0.1526;%!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!STARTING ANGLE OFFSET 0.15 RAD for flight 12 
%aligning drone and optitrack heading at the start

psi_int = OPTITRACK_OFFSET;
filter_psi = OPTITRACK_OFFSET;
turn_offset = 0;

psi_grad_g1 = 0;
psi_grad_g2 = 0;
psi_p3p_g1 = 0;
psi_p3p_g2 = 0;

min_detection_quality = 0.90;%0.85;%0.90;%0.85;
psi_filter_gain = 0.1;%0.05;%0.2;

run_vision_heading = 1;

%psi optitrack integration
psi_1 = ones(data_size,1)*OPTITRACK_OFFSET;
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


if(run_vision_heading == 0)
    load('v_psi_measurements.mat')
end

if run_processing
    
for n = 2:data_size
   
    dt = time_stamp(n)-time_stamp(n-1);
    
   body_speed(n,:) = (C_b_n(phi(n),theta(n),psi(n))'*[vel_x(n) vel_y(n) vel_z(n)]')';
   
   %integrated psi
   eul_d = Kin_att( theta(n), phi(n) )*([gyro_p(n) gyro_q(n) gyro_r(n)]-gyro_bias)';
   psi_int(n) = psi_int(n-1) + eul_d(3)*dt;
   filter_psi(n) = filter_psi(n-1) + eul_d(3)*dt;
    
   turn_offset(n) = round(filter_psi(n)/pi)*pi;
    
    %add 30 deg yaw error after first turn
    psi_error_true = 0;
    error_point = 320;
    error_step = 30;
    if(filter_psi(n) > deg2rad(error_point) && filter_psi(n-1) < deg2rad(error_point) && psi_error_true)
        filter_psi(n) = filter_psi(n) + deg2rad(error_step);
    end
    
   total_diff_c = corner_1_x(n) - corner_1_x(n-1) + corner_1_y(n) - corner_1_y(n-1) ...
    + corner_2_x(n) - corner_2_x(n-1) + corner_2_y(n) - corner_2_y(n-1) ...
    + corner_3_x(n) - corner_3_x(n-1) + corner_3_y(n) - corner_3_y(n-1) ...
    + corner_4_x(n) - corner_4_x(n-1) + corner_4_y(n) - corner_4_y(n-1);

    total_diff_s = 0;%no histogram needed
    
    if(psi(n) > 1.6 || psi(n) < -1.6)
        if(psi(n) < 0)
            local_psi = psi(n)+3.14;
        else
            local_psi = psi(n)-3.14;
        end
    else
        local_psi = psi(n);
    end
    
    
  selected_pose_ls = [0 0 0]';
  
    if (corner_1_x(n) ~= 0 && abs(total_diff_c) > 1)
        
        detect_count = detect_count + 1;
        
        prev_x_1 = corner_1_x(n);
        prev_y_1 = corner_1_y(n);
        prev_x_2 = corner_2_x(n);
        prev_y_2 = corner_2_y(n);
        prev_x_3 = corner_3_x(n);
        prev_y_3 = corner_3_y(n);
        prev_x_4 = corner_4_x(n);
        prev_y_4 = corner_4_y(n);
        
        ts =  time_stamp(n)
   
        opti_pos = [pos_x(n) pos_y(n) pos_z(n)];

        gate_angle_offset = deg2rad(0);

        time_stamp_side(detect_count) = time_stamp(n);

        gate_2_offset = deg2rad(0);

        if(abs(psi(n))>deg2rad(90))
        image_points_x = [corner_1_x(n) corner_2_x(n) corner_3_x(n) corner_4_x(n)];
        image_points_y = [corner_1_y(n) corner_2_y(n) corner_3_y(n) corner_4_y(n)];

        if(matlab_undistort)
        [corX, corY] = undistort_points_fish_eye_5(image_points_x,image_points_y,cameraParams_rot_3);
        else
        [corX, corY] = undistort_points_fish_eye_2(image_points_x,image_points_y);
        end
        selected_pose_ls;
             if(rad2deg(psi(n))<0)
                psi_2 = psi(n)+deg2rad(180);
             else
                psi_2 = psi(n)-deg2rad(180);
             end
             [selected_pose_ls] = pos_from_att_ned_20(gate_points,cam_pos,corX-x_p,corY-y_p,-phi(n), theta(n), -psi_2+deg2rad(0));            

            pos_x_ls(detect_count) = 3.0-selected_pose_ls(1);
            pos_y_ls(detect_count) = -selected_pose_ls(2)+3.25;
            pos_z_ls(detect_count) = -selected_pose_ls(3)-0.1;

             psi_disturbance = deg2rad(0);
            if(gate_quality(n) > min_detection_quality && run_vision_heading)
                detect_count_a = detect_count_a + 1;
                time_stamp_a(detect_count_a) = time_stamp(n);
                psi_gt(detect_count_a) = rad2deg(psi_1(n));
                
               [pos_a, phi_est, theta_est, psi_est] = angle_from_pos_grad_2(gate_pos_neu,[selected_pose_ls(1) selected_pose_ls(2) sonar_alt(n)],corX-x_p,corY-y_p,-phi(n),-theta(n),0+psi_disturbance);
                
                vision_psi(detect_count_a) = psi_est + gate_2_offset + turn_offset(n);
                psi_grad_g2 = [psi_grad_g2; psi_est];
                
                [ selected_pose_av, best_R_p3p ] = pos_p3p_ransac_neu(gate_pos_neu, corX, corY)
            
                dcm00 = best_R_p3p(1,1);
                dcm01 = best_R_p3p(1,2);
                dcm02 = best_R_p3p(1,3);
                dcm12 = best_R_p3p(2,3);
                dcm22 = best_R_p3p(3,3);

                phi_R = rad2deg( atan2(dcm12, dcm22));
                theta_R = rad2deg( -asin(dcm02));
                psi_R = rad2deg(atan2(dcm01, dcm00))

                %bound psi_R
                 if(psi_R > 30)
                    psi_R = 30;
                 elseif(psi_R < -30)
                    psi_R = -30;
                 end
                
                vision_psi_p3p(detect_count_a) = deg2rad(psi_R) + gate_2_offset + turn_offset(n);
                psi_p3p_g2 = [psi_p3p_g2; deg2rad(psi_R)];
                 
                %psi correction
                 filter_psi(n) = filter_psi(n) - psi_filter_gain*(filter_psi(n)-vision_psi(detect_count_a));
%                filter_psi(n) = filter_psi(n) - psi_filter_gain*(filter_psi(n)-vision_psi_p3p(detect_count_a));
                 
            end

        end

        if(abs(psi(n))<deg2rad(90))%
        image_points_x = [corner_1_x(n) corner_2_x(n) corner_3_x(n) corner_4_x(n)];
        image_points_y = [corner_1_y(n) corner_2_y(n) corner_3_y(n) corner_4_y(n)];

        if(matlab_undistort)
          [corX, corY] = undistort_points_fish_eye_5(image_points_x,image_points_y,cameraParams_rot_3);
        else
        [corX, corY] = undistort_points_fish_eye_2(image_points_x,image_points_y);
        end
        
        n
        position = [pos_x(n) pos_y(n) pos_z(n)];

        selected_pose_ls;

        psi_disturbance = deg2rad(0);
        selected_pose_ls = pos_from_att_ned_20(gate_points,cam_pos,corX-x_p,corY-y_p,-phi(n), theta(n), -psi(n)+psi_disturbance);            

        pos_x_ls(detect_count) = selected_pose_ls(1);
        pos_y_ls(detect_count) = selected_pose_ls(2)-0.05;
        pos_z_ls(detect_count) = -selected_pose_ls(3);
 
        gate_1_offset = deg2rad(0);
        if(gate_quality(n) > min_detection_quality && run_vision_heading)
            
            [pos_a, phi_est, theta_est, psi_est] = angle_from_pos_grad_2(gate_pos_neu,[selected_pose_ls(1) selected_pose_ls(2) sonar_alt(n)],corX-x_p,corY-y_p,-phi(n),-theta(n),0+psi_disturbance);
                       
            pos_estimation = [selected_pose_ls(1) selected_pose_ls(2) sonar_alt(n)]
             psi_estimate = rad2deg(psi_est)

             detect_count_a = detect_count_a + 1;
             psi_gt(detect_count_a) = rad2deg(psi_1(n));
             time_stamp_a(detect_count_a) = time_stamp(n);
             vision_psi(detect_count_a) = psi_est + gate_1_offset + turn_offset(n);
             v_psi_error(detect_count_a) = vision_psi(detect_count_a)-psi(n);
             psi_grad_g1 = [psi_grad_g1; psi_est];
             
             cameraParams = 1;
              [ selected_pose_av, best_R_p3p ] = pos_p3p_ransac_neu(gate_pos_neu, corX, corY)
%             [ selected_pose_av, best_R_p3p ] = pos_p3p_ransac_Gao_neu(gate_pos_neu, corX, corY)
            
            dcm00 = best_R_p3p(1,1);
            dcm01 = best_R_p3p(1,2);
            dcm02 = best_R_p3p(1,3);
            dcm12 = best_R_p3p(2,3);
            dcm22 = best_R_p3p(3,3);

            phi_R = rad2deg( atan2(dcm12, dcm22));
            theta_R = rad2deg( -asin(dcm02));
            psi_R = rad2deg(atan2(dcm01, dcm00))

             %bound psi_R
             if(psi_R > 30)
                 psi_R = 30;
             elseif(psi_R < -30)
                 psi_R = -30;
             end
                 
             vision_psi_p3p(detect_count_a) = deg2rad(psi_R) + gate_1_offset  + turn_offset(n);
             v_psi_error_p3p(detect_count_a) = vision_psi_p3p(detect_count_a)-psi(n);
             psi_p3p_g1 = [psi_p3p_g1; deg2rad(psi_R)];
             
             %psi correction
              filter_psi(n) = filter_psi(n) - psi_filter_gain*(filter_psi(n)-vision_psi(detect_count_a));
%             filter_psi(n) = filter_psi(n) - psi_filter_gain*(filter_psi(n)-vision_psi_p3p(detect_count_a));

             [ selected_pose_Gao, best_R_p3p ] = pos_p3p_ransac_Gao_neu(gate_pos_neu, corX, corY)
             
            dcm00 = best_R_p3p(1,1);
            dcm01 = best_R_p3p(1,2);
            dcm02 = best_R_p3p(1,3);
            dcm12 = best_R_p3p(2,3);
            dcm22 = best_R_p3p(3,3);

            selected_pose_av;
            best_R_p3p;
            phi_R = rad2deg( atan2(dcm12, dcm22));
            theta_R = rad2deg( -asin(dcm02))
            psi_R = rad2deg(atan2(dcm01, dcm00))
           
            vision_psi_p3p_g(detect_count_a) = deg2rad(psi_R) + gate_1_offset + turn_offset(n);
            vision_theta_p3p_g(detect_count_a) = deg2rad(theta_R)
            v_psi_error_p3p_g(detect_count_a) = vision_psi_p3p_g(detect_count_a)-psi(n);
                   
        end
             
        end
   
    end
    
    if(abs(total_diff_c) > 1 || abs(total_diff_s) > 0)
        
        min_dist = 0.4;
        max_dist = 2.8;
        %sd_dist = 0; %dummy data for bad log file
        sd_dist = x_pos_hist(n);%gate_d;
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

    end
    
     ls_position = [ls_position;selected_pose_ls'];
end
end

detect_count
detect_count_a

if run_vision_heading
 squareError = v_psi_error.^2;
 meanSquareError = mean(squareError);
 RMSE_psi_v = sqrt(meanSquareError)
 VARIANCE_psi_v = var(v_psi_error)
 
 squareError = v_psi_error_p3p.^2;
 meanSquareError = mean(squareError);
 RMSE_psi_v_p3p = sqrt(meanSquareError)
 VARIANCE_psi_v_p3p = var(v_psi_error_p3p)
end


figure()
% plot3(pose_x,pose_y,pose_z)
grid on;
hold on;
plot3(pos_x,pos_y,-pos_z)
hold on
plot3(pos_x_ls,pos_y_ls,pos_z_ls,'+')
axis([-3 6, -2 4, 0 3.5])

figure()
plot(time_stamp,pos_x)
hold on
plot(time_stamp_m,pos_vision_x,'+')
axis([0 70,-3 6])
%axis([27.2 29.2, 1 3])
legend('Optitrack x','Vision pos')
title('X position')

figure()
plot(time_stamp,gate_quality)
title('gate_quality')

%plot article
%sample optitrack data at on-board log rate
psi_op = zeros(data_size,1);
figure()
if(gyro_heading_flight)
%     plot(opti_data.TIME+time_shift,opti_data.PSI+26.65)
%     hold on
    for n = 2:data_size
        %[val,idx] = min(abs(opti_data.TIME+time_shift-time_stamp(n)));
        psi_op(n) = psi_1(n);%deg2rad(opti_data.PSI(idx)+26.65);%!!!!!!!!!!!!!!!!!PSI OP DIRECTLY FROM ONBOARD DRONE
    end
end

plot(time_stamp,rad2deg(psi_op(:)))
% hold on
% plot(time_stamp,rad2deg(psi_1(:)))
hold on
plot(time_stamp,rad2deg(filter_psi(:)))
hold on
plot(time_stamp,rad2deg(psi_int(:)))
hold on
plot(time_stamp_a,rad2deg(vision_psi),'+')
% hold on
% plot(time_stamp_a,rad2deg(vision_psi_p3p),'+')
%axis([95 100,2660 2730])
title('Psi angle after 28 gate passes')
if(gyro_heading_flight)
%     legend('Psi ground truth','Psi on-board','Psi filtered','Psi integrated','Vision psi measurements','Vision psi P3P')
      legend('Psi optitrack','Psi filtered','Psi integrated','Vision psi measurements')
else
    legend('Psi ground truth','Psi filtered','Psi integrated','Vision psi measurements','Vision psi P3P')
end
%%axis([55 63, 1010 1150])
axis([192 200, 4960 5100])%used in article
xlabel('Time [sec]') % x-axis label
ylabel('Psi [deg]') % y-axis label
saveas(gcf,'../autosave/png/vision_psi_28.png')
saveas(gcf,'../autosave/eps/vision_psi_28','epsc')


figure()
plot(time_stamp,rad2deg(psi_op-filter_psi'))
hold on
plot(time_stamp,rad2deg(psi_op-psi_int'))
title('Psi error')
legend('Psi filter','Psi integrated')
axis([0 213, -25 15])
xlabel('Time [sec]') % x-axis label
ylabel('Psi error [deg]') % y-axis label
saveas(gcf,'../autosave/png/psi_error_estimate_28.png')
saveas(gcf,'../autosave/eps/psi_error_estimate_28','epsc')



