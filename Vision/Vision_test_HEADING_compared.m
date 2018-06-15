close all;
clear all;

angle_to_gate = deg2rad(0);
pixel_noise = 3.5;%
angle_noise = deg2rad(3.8);%deg2rad(10);%in deg

RMSE = zeros(3,9)
RMSE_psi_p3p = zeros(3,9)
RMSE_psi_grad = zeros(3,9)
RMSE_psi_ga = zeros(3,9)
RMSE_X_ls0 = zeros(3,9)
RMSE_Y_ls0 = zeros(3,9)
RMSE_Z_ls0 = zeros(3,9)

dist_ = zeros(1,9);
sample_size = 100;%
for angle = 1:3%
    angle_deg = (angle*30)-60;
    angle_to_gate = deg2rad(angle_deg);
    
    for distance = 1:9%
      distance_to_gate = 4.2 - (distance/3);
      dist_(distance) = distance_to_gate;
      av_size = sample_size;%
      total_error = zeros(av_size,1);
      total_error_psi_p3p = zeros(av_size,1);
      total_error_x_ls0 = zeros(av_size,1);
      total_error_y_ls0 = zeros(av_size,1);
      total_error_z_ls0 = zeros(av_size,1);
      
      for n = 1:av_size
       gate_pos = [4.2 0.8 1.4];%
       [psi_error_p3p] = Vision_error_sim_psi_p3p(angle_to_gate,distance_to_gate,gate_pos,pixel_noise);

       pos_error_ls0 = [0 0 0];
       total_error_psi_p3p(n) = rad2deg(psi_error_p3p);
       total_error_x_ls0(n) = pos_error_ls0(1);
       total_error_y_ls0(n) = pos_error_ls0(2);
       total_error_z_ls0(n) = pos_error_ls0(3);

      end
      squareError = total_error.^2;
      meanSquareError = mean(squareError);
      RMSE(angle,distance) = sqrt(meanSquareError)
      RMSE_psi_p3p(angle,distance) = sqrt(mean(total_error_psi_p3p.^2))
      RMSE_X_ls0(angle,distance) = sqrt(mean(total_error_x_ls0.^2));
      RMSE_Y_ls0(angle,distance) = sqrt(mean(total_error_y_ls0.^2));
      RMSE_Z_ls0(angle,distance) = sqrt(mean(total_error_z_ls0.^2));
      
    end
    
end

dist_ = zeros(1,9);
sample_size = 100;%
for angle = 1:3%
    angle_deg = (angle*30)-60;
    angle_to_gate = deg2rad(angle_deg);
    
    for distance = 1:9%
        angle
        distance
      distance_to_gate = 4.2 - (distance/3);
      dist_(distance) = distance_to_gate;
      av_size = sample_size;%
      total_error = zeros(av_size,1);
      total_error_psi_grad = zeros(av_size,1);
      
      for n = 1:av_size
       gate_pos = [4.2 0.8 1.4];%
       [psi_error_grad] = Vision_error_sim_psi_grad(angle_to_gate,distance_to_gate,gate_pos,pixel_noise,angle_noise);
       total_error_psi_grad(n) = rad2deg(psi_error_grad);

      end

      squareError = total_error.^2;
      meanSquareError = mean(squareError);
      RMSE(angle,distance) = sqrt(meanSquareError)
      RMSE_psi_grad(angle,distance) = sqrt(mean(total_error_psi_grad.^2))
      
    end
    
end

dist_ = zeros(1,9);
sample_size = 100;%
for angle = 1:3%
    angle_deg = (angle*30)-60;
    angle_to_gate = deg2rad(angle_deg);
    
    for distance = 1:9%
        angle
        distance
      distance_to_gate = 4.2 - (distance/3);
      dist_(distance) = distance_to_gate;
      av_size = sample_size;%was 1000
      total_error = zeros(av_size,1);
      total_error_psi_ga = zeros(av_size,1);
      
      for n = 1:av_size
       gate_pos = [4.2 0.8 1.4];%
       [psi_error_ga] = Vision_error_sim_psi_ga(angle_to_gate,distance_to_gate,gate_pos,pixel_noise,angle_noise);
       total_error_psi_ga(n) = rad2deg(psi_error_ga);
      end
      
      squareError = total_error.^2;
      meanSquareError = mean(squareError);
      RMSE(angle,distance) = sqrt(meanSquareError)
      RMSE_psi_ga(angle,distance) = sqrt(mean(total_error_psi_ga.^2))
      
    end
    
end


figure()
plot(dist_,RMSE_psi_p3p(2,:),'-*')
hold on
plot(dist_,RMSE_psi_grad(2,:),'-+')
hold on
plot(dist_,RMSE_psi_ga(2,:),'-o')

grid on
legend('P3p','Gradient','Genetic')
xlabel('Distance to gate [M]') % x-axis label
ylabel('Psi RMSE [deg]') % y-axis label
%title('Psi RMSE 0 deg')
axis([1 4, 0 12])
x0=10;
y0=10;
width=250;
height=200;
set(gca,'FontSize',8)
set(gcf,'units','points','position',[x0,y0,width,height])
saveas(gcf,'../autosave/png/psi_RMSE_0_deg_100.png')
saveas(gcf,'../autosave/eps/psi_RMSE_0_deg_100','epsc')


figure()
plot(dist_,RMSE_psi_p3p(1,:),'-*')
hold on
plot(dist_,RMSE_psi_grad(1,:),'-+')
hold on
plot(dist_,RMSE_psi_ga(1,:),'-o')

grid on
legend('P3p','Gradient','Genetic')
xlabel('Distance to gate [M]') % x-axis label
ylabel('Psi RMSE [deg]') % y-axis label
%title('Psi RMSE -30 deg')
axis([1 4, 0 12])
x0=10;
y0=10;
width=250;
height=200;
set(gca,'FontSize',8)
set(gcf,'units','points','position',[x0,y0,width,height])
saveas(gcf,'../autosave/png/psi_RMSE_n30_deg_100.png')
saveas(gcf,'../autosave/eps/psi_RMSE_n30_deg_100','epsc')

figure()
plot(dist_,RMSE_psi_p3p(3,:),'-*')
hold on
plot(dist_,RMSE_psi_grad(3,:),'-+')
hold on
plot(dist_,RMSE_psi_ga(3,:),'-o')
grid on
legend('P3p','Gradient','Genetic')
xlabel('Distance to gate [M]') % x-axis label
ylabel('Psi RMSE [deg]') % y-axis label
%title('Psi RMSE 30 deg')
axis([1 4, 0 12])
x0=10;
y0=10;
width=250;
height=200;
set(gca,'FontSize',8)
set(gcf,'units','points','position',[x0,y0,width,height])
saveas(gcf,'../autosave/png/psi_RMSE_30_deg_100.png')
saveas(gcf,'../autosave/eps/psi_RMSE_30_deg_100','epsc')


