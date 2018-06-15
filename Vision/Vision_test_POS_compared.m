close all;

angle_to_gate = deg2rad(0);
pixel_noise = 3.5;%
angle_noise = deg2rad(0);%in deg
sample_size = 1000;

RMSE = zeros(3,10)
RMSE_X_p3p = zeros(3,10)
RMSE_Y_p3p = zeros(3,10)
RMSE_Z_p3p = zeros(3,10)
RMSE_X_ls0 = zeros(3,10)
RMSE_Y_ls0 = zeros(3,10)
RMSE_Z_ls0 = zeros(3,10)
RMSE_X_ls5 = zeros(3,10)
RMSE_Y_ls5 = zeros(3,10)
RMSE_Z_ls5 = zeros(3,10)
RMSE_X_ls15 = zeros(3,10)
RMSE_Y_ls15 = zeros(3,10)
RMSE_Z_ls15 = zeros(3,10)

dist_ = zeros(1,10);

for angle = 1:3%
    angle_deg = (angle*30)-60;
    angle_to_gate = deg2rad(angle_deg);
    
    for distance = 1:10
      distance_to_gate = 4.2 - (distance/3);
      dist_(distance) = distance_to_gate;
      av_size = sample_size;%was 1000
      total_error = zeros(av_size,1);
      total_error_x_p3p = zeros(av_size,1);
      total_error_y_p3p = zeros(av_size,1);
      total_error_z_p3p = zeros(av_size,1);
      total_error_x_ls0 = zeros(av_size,1);
      total_error_y_ls0 = zeros(av_size,1);
      total_error_z_ls0 = zeros(av_size,1);
      total_error_x_ls5 = zeros(av_size,1);
      total_error_y_ls5 = zeros(av_size,1);
      total_error_z_ls5 = zeros(av_size,1);
      total_error_x_ls15 = zeros(av_size,1);
      total_error_y_ls15 = zeros(av_size,1);
      total_error_z_ls15 = zeros(av_size,1);
      for n = 1:av_size
        gate_pos = [4.2 0.8 -1.4];%
        [pos_error_p3p] = Vision_error_sim(angle_to_gate,distance_to_gate,gate_pos,pixel_noise);
        angle_noise = deg2rad(0);
        [pos_error_ls0] = Vision_error_sim_LS(angle_to_gate,distance_to_gate,gate_pos,pixel_noise,angle_noise);
        angle_noise = deg2rad(2.2);%var 5 deg
        [pos_error_ls5] = Vision_error_sim_LS(angle_to_gate,distance_to_gate,gate_pos,pixel_noise,angle_noise);
        angle_noise = deg2rad(3.8);%var 15 deg
        [pos_error_ls15] = Vision_error_sim_LS(angle_to_gate,distance_to_gate,gate_pos,pixel_noise,angle_noise);
       total_error(n) = sqrt(sum(pos_error_p3p.^2)); %euclidean norm
       total_error_x_p3p(n) = pos_error_p3p(1);
       total_error_y_p3p(n) = pos_error_p3p(2);
       total_error_z_p3p(n) = pos_error_p3p(3);
       total_error_x_ls0(n) = pos_error_ls0(1);
       total_error_y_ls0(n) = pos_error_ls0(2);
       total_error_z_ls0(n) = pos_error_ls0(3);
       total_error_x_ls5(n) = pos_error_ls5(1);
       total_error_y_ls5(n) = pos_error_ls5(2);
       total_error_z_ls5(n) = pos_error_ls5(3);
       total_error_x_ls15(n) = pos_error_ls15(1);
       total_error_y_ls15(n) = pos_error_ls15(2);
       total_error_z_ls15(n) = pos_error_ls15(3);
      end
      squareError = total_error.^2;
      meanSquareError = mean(squareError);
      RMSE(angle,distance) = sqrt(meanSquareError)
      RMSE_X_p3p(angle,distance) = sqrt(mean(total_error_x_p3p.^2))
      RMSE_Y_p3p(angle,distance) = sqrt(mean(total_error_y_p3p.^2))
      RMSE_Z_p3p(angle,distance) = sqrt(mean(total_error_z_p3p.^2))
      RMSE_X_ls0(angle,distance) = sqrt(mean(total_error_x_ls0.^2))
      RMSE_Y_ls0(angle,distance) = sqrt(mean(total_error_y_ls0.^2))
      RMSE_Z_ls0(angle,distance) = sqrt(mean(total_error_z_ls0.^2))
      RMSE_X_ls5(angle,distance) = sqrt(mean(total_error_x_ls5.^2))
      RMSE_Y_ls5(angle,distance) = sqrt(mean(total_error_y_ls5.^2))
      RMSE_Z_ls5(angle,distance) = sqrt(mean(total_error_z_ls5.^2))
      RMSE_X_ls15(angle,distance) = sqrt(mean(total_error_x_ls15.^2))
      RMSE_Y_ls15(angle,distance) = sqrt(mean(total_error_y_ls15.^2))
      RMSE_Z_ls15(angle,distance) = sqrt(mean(total_error_z_ls15.^2))
    end
    
end

figure()
plot(dist_,RMSE_X_p3p(2,:),'-*')
hold on
plot(dist_,RMSE_X_ls0(2,:),'-o')
hold on
plot(dist_,RMSE_X_ls5(2,:),'-+')
hold on
plot(dist_,RMSE_X_ls15(2,:),'-x')
grid on
legend('P3p','LS var 0 deg','LS var 5 deg','LS var 15 deg')
xlabel('Distance to gate [M]') % x-axis label
ylabel('X RMSE [M]') % y-axis label
%title('X Position RMSE')
axis([0 4, 0 0.7])
x0=10;
y0=10;
width=250;
height=200;
set(gca,'FontSize',8) % Creates an axes and sets its FontSize to 8
set(gcf,'units','points','position',[x0,y0,width,height])
saveas(gcf,'../autosave/png/X_pos_RMSE_2.png')
saveas(gcf,'../autosave/eps/X_pos_RMSE_2','epsc')


figure()
plot(dist_,RMSE_Y_p3p(2,:),'-*')
hold on
plot(dist_,RMSE_Y_ls0(2,:),'-o')
hold on
plot(dist_,RMSE_Y_ls5(2,:),'-+')
hold on
plot(dist_,RMSE_Y_ls15(2,:),'-x')
grid on
legend('P3p','LS var 0 deg','LS var 5 deg','LS var 15 deg')
xlabel('Distance to gate [M]') % x-axis label
ylabel('Y RMSE [M]') % y-axis label
%title('Y Position RMSE')
axis([0 4, 0 0.7])
x0=20;
y0=10;
width=250;
height=200;
set(gca,'FontSize',8)
set(gcf,'units','points','position',[x0,y0,width,height])
saveas(gcf,'../autosave/png/Y_pos_RMSE_2.png')
saveas(gcf,'../autosave/eps/Y_pos_RMSE_2','epsc')


figure()
plot(dist_,RMSE_Z_p3p(2,:),'-*')
hold on
plot(dist_,RMSE_Z_ls0(2,:),'-o')
hold on
plot(dist_,RMSE_Z_ls5(2,:),'-+')
hold on
plot(dist_,RMSE_Z_ls15(2,:),'-x')
grid on
legend('P3p','LS var 0 deg','LS var 5 deg','LS var 15 deg')
xlabel('Distance to gate [M]') % x-axis label
ylabel('Z RMSE [M]') % y-axis label
%title('Z Position RMSE')
axis([0 4, 0 0.7])
x0=30;
y0=10;
width=250;
height=200;
set(gca,'FontSize',8)
set(gcf,'units','points','position',[x0,y0,width,height])
saveas(gcf,'../autosave/png/Z_pos_RMSE_2.png')
saveas(gcf,'../autosave/eps/Z_pos_RMSE_2','epsc')


