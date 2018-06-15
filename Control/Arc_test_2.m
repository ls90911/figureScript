
close all
clear all

VarName = csvread('Inf_loop_tests/session_6/flight_4.csv');
opti_data = import_optitrack_data_2('Inf_loop_tests/session_6/Take 2017-12-20 07.11.41 PM_f4',{'RigidBody 4'})
time_shift = 6.5;
% %THIS_used_for_arc_accuracy-------------------------------

counter = VarName(:,1); time_stamp = VarName(:,2);

%gyro scale 0.0139882 for deg/sec
gyro_scale = deg2rad(0.0139882);

%acc scale 0.0009766 for m/s2
acc_scale = 0.0009766;


gyro_p = VarName(:,3)*gyro_scale; gyro_q = VarName(:,4)*gyro_scale; gyro_r = VarName(:,5)*gyro_scale;
acc_x = VarName(:,6)*acc_scale; acc_y = VarName(:,7)*acc_scale; acc_z = VarName(:,8)*acc_scale;
phi = VarName(:,12); theta = VarName(:,13); psi = VarName(:,14);
pos_x = VarName(:,15); pos_y = VarName(:,16); pos_z = VarName(:,17);
vel_x_raw = VarName(:,18); vel_y_raw = VarName(:,19); vel_z_raw = VarName(:,20);

vel_x = movmean(vel_x_raw,60); 
vel_y = movmean(vel_y_raw,60); 
vel_z = movmean(vel_z_raw,60); 

corner_1_x = VarName(:,36); corner_1_y = VarName(:,37);
corner_2_x = VarName(:,38); corner_2_y = VarName(:,39);
corner_3_x = VarName(:,40); corner_3_y = VarName(:,41);
corner_4_x = VarName(:,42); corner_4_y = VarName(:,43);

mean_speed = VarName(:,61);

[data_size, ~ ] = size(counter);
[opti_size, ~ ] = size(opti_data.TIME);
in_turn = 0;

start_1 = [0 0];
start_2 = [0 0];
end_1 = [0 0];
end_2 = [0 0];
error_1 = [0 0];
error_2 = [0 0];
start_vb_1 = [0 0];
start_vb_2 = [0 0];
start_vb_ob_1 = 0;
start_vb_ob_2 = 0;
start_vbd_y_1 = 0;
end_vb_1 = [0 0];
end_vb_2 = [0 0];

body_speed = zeros(data_size,3);
thrust_angle = zeros(data_size,1);

%matlab 3 coeficient radial distortion correction

 x_p = 315-152.8;
 y_p = 35.7;
 detect_count = 0;

gate_pos = [3.5 0.05 -1.4];
gate_pos_neu = [3.5 0.05 1.4];
cam_pos = [0 0 0];%unknown dummy


mean_speed_d = [diff(mean_speed,2); 0.1; 0.1];
psi_diff = [diff(psi); 0.1];

%psi integration
psi_1 = zeros(data_size,1);

for i = 2:data_size
    psi_delta = psi(i)-psi(i-1);
    if(psi_delta < -pi)
        psi_1(i) = psi_1(i-1) + psi_delta + 2*pi;
    elseif(psi_delta > pi)
        psi_1(i) = psi_1(i-1) + psi_delta - 2*pi;
    else
        psi_1(i) = psi_1(i-1) + psi_delta;
    end
end

%acc drag
fc = 5; % Cut off frequency
fs = 512; % Sampling rate

[b,a] = butter(6,fc/(fs/2)); % Butterworth filter of order 6
acc_x_filter = movmean(acc_x,[512*2 1]);
acc_y_filter = filter(b,a,acc_y); 
K1 = -0.55;%
K2 = -0.9;%
acc_bias_y = 0.0;
acc_y_drag_speed = (acc_y_filter-acc_bias_y)/K2;

for n = 51:data_size
    
   dt = time_stamp(n)-time_stamp(n-1);
   body_speed(n,:) = (C_b_n(phi(n),theta(n),psi(n))'*[vel_x(n) vel_y(n) vel_z(n)]')';
   
   %use optitrack attitude
   thrust_vec = (C_b_n(phi(n),theta(n),psi(n))'*[0 0 -1]')';
   down_vec = [0 0 -1]';
   thrust_angle(n) = acos(dot(thrust_vec, down_vec) / (norm(thrust_vec) * norm(down_vec)));
   
   %matlab attitude filter

    
 phi_diff = phi(n)-phi(n-50);
 angle_threshold = 6;
 %first turn accuracy
if(pos_x(n) > 1.6 && pos_y(n) < 1.5 && in_turn == 0 && abs(psi_diff(n)) > 1e-3 && abs(psi_diff(n)) < 4e-3 && phi(n) > deg2rad(angle_threshold)&& psi(n) > deg2rad(2))%abs(mean_speed_d(n)-mean_speed_d(n-10)) < 1e-10)% phi(n) > deg2rad(angle_threshold) && psi(n) > deg2rad(2))
    start_x = pos_x(n)
    start_y = pos_y(n)
    start_1 = [start_1; start_x start_y];
    start_vb_1 = [start_vb_1; body_speed(n,1) body_speed(n,2)];
    start_vb_ob_1 = [start_vb_ob_1;mean_speed(n)];
    start_vbd_y_1 = [start_vbd_y_1;acc_y_drag_speed(n)];
    in_turn = 1;
end

if(pos_x(n) < 1.6 && pos_y(n) > 1.5 && in_turn == 0 && abs(psi_diff(n)) > 1e-3 && abs(psi_diff(n)) < 4e-3 &&( phi(n) > deg2rad(angle_threshold) && psi(n) < deg2rad(178) && psi(n) > deg2rad(-178)))
    start_x = pos_x(n)
    start_y = pos_y(n)
    start_2 = [start_2; start_x start_y];
    start_vb_2 = [start_vb_2; body_speed(n,1) body_speed(n,2)];
    start_vb_ob_2 = [start_vb_ob_2;mean_speed(n)];
    in_turn = 2;
end


if((phi(n) < deg2rad(8) && psi(n) > deg2rad(178) ||(pos_y(n)-pos_y(n-1))<0) && in_turn == 1)
    %end turn 1
    end_1 = [end_1; pos_x(n) pos_y(n)];
    error_1 = [error_1; pos_x(n)-start_x (pos_y(n)-start_y)-3];
    end_vb_1 = [end_vb_1; body_speed(n,1) body_speed(n,2)];
    in_turn = 0;
elseif(((phi(n) < deg2rad(8) && psi(n) > deg2rad(-2)) ||(pos_y(n)-pos_y(n-1))>0) && in_turn == 2)
    %end turn 2
    error_2 = [error_2; pos_x(n)-start_x (pos_y(n)-start_y)+3];
    end_2 = [end_2; pos_x(n) pos_y(n)];
    end_vb_2 = [end_vb_2; body_speed(n,1) body_speed(n,2)];
    in_turn = 0;
end


end

%gate at origin
g_1_x = 3.9;
g_1_y = -0.50;
g_2_x = 0;
g_2_y = 2.7;
test_3d_plot = [0+g_1_x 0+g_1_y 0; 0+g_1_x 0+g_1_y 1;0+g_1_x -0.5+g_1_y 1;0+g_1_x -0.5+g_1_y 2;0+g_1_x 0.5+g_1_y 2;0+g_1_x 0.5+g_1_y 1;0+g_1_x 0+g_1_y 1];
test_3d_plot_2 = [0+g_2_x 0+g_2_y 0; 0+g_2_x 0+g_2_y 1;0+g_2_x -0.5+g_2_y 1;0+g_2_x -0.5+g_2_y 2;0+g_2_x 0.5+g_2_y 2;0+g_2_x 0.5+g_2_y 1;0+g_2_x 0+g_2_y 1];

start_plotting = 2000;
[end_ , ~]= size(pos_x);


figure()
plot3(pos_x,pos_y,-pos_z)
hold on
plot3(start_1(:,1),start_1(:,2),ones(size(start_1(:,2)))*1.4,'+')
hold on
plot3(start_2(:,1),start_2(:,2),ones(size(start_2(:,2)))*1.4,'+')
hold on
plot3(end_1(:,1),end_1(:,2),ones(size(end_1(:,2)))*1.4,'+')
hold on
plot3(end_2(:,1),end_2(:,2),ones(size(end_2(:,2)))*1.4,'+')
hold on
plot3(test_3d_plot(:,1),test_3d_plot(:,2),test_3d_plot(:,3),'Linewidth',3,'Color','r')
hold on
plot3(test_3d_plot_2(:,1),test_3d_plot_2(:,2),test_3d_plot_2(:,3),'Linewidth',3,'Color','r')
grid on
axis([-3 7, -2 5, 0 3.5])

figure()
plot(pos_x(1:100000),pos_y(1:100000))
hold on
plot(start_1(:,1),start_1(:,2),'+')
hold on
plot(end_1(:,1),end_1(:,2),'o')
hold on
X = [1.5, 3.5]
Y = [4.5, 4.5];
basevalue = -1;
ar = area(X,Y,basevalue);
ar(1).FaceColor = [1 1 1];
axis([1.5 6.5 -1 4.5])
legend('Optitrack','Arc entry','Arc exit')
title('Arc accuracy')
xlabel('X position [M]') % x-axis label
ylabel('Y position [M]') % y-axis label
saveas(gcf,'../autosave/png/Arc_accuracy_1.png')
saveas(gcf,'../autosave/eps/Arc_accuracy_1','epsc')

entry_v_x_1 = start_vb_ob_1;
entry_v_y_1 = start_vbd_y_1;
entry_v_x_2 = start_vb_ob_2;
entry_v_y_2 = start_vb_2(:,2)';
error_v_x_1 = entry_v_x_1 - start_vb_1(:,1)
error_v_y_1 = entry_v_y_1 - start_vb_1(:,2)
delta_x_1 = (end_1(:,1)-start_1(:,1))'
delta_x_2 = (end_2(:,1)-start_2(:,1))'
delta_y_1 = (end_1(:,2)-start_1(:,2))'
delta_y_2 = (end_2(:,2)-start_2(:,2))'

VAR_X_ERROR = var(error_v_x_1)
VAR_Y_ERROR = var(error_v_y_1)
VAR_X_DELTA = var(delta_x_1)
VAR_Y_DELTA = var(delta_y_1)


figure
plot(entry_v_x_1,'-+')
hold on
plot(entry_v_y_1*10,'-+')
hold on
plot(delta_y_1,'-+')
hold on
legend('X entry speed','Y entry speed','Y delta')
title('Turn 1')

figure
plot(entry_v_x_2,'-+')
hold on
plot(entry_v_y_2*10,'-+')
hold on
plot(-delta_y_2,'-+')
hold on
legend('X entry speed','Y entry speed','Y delta')
title('Turn 2')


