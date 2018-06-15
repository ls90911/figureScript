close all
clear all

gate_size = 1;
gate_pos = [0 0];
s_1 = [gate_pos(1) gate_pos(2)+gate_size/2];
s_2 = [gate_pos(1) gate_pos(2)-gate_size/2];

sample_size = 1000;
dist_ = zeros(1,9);

RMSE_X = zeros(3,9)
RMSE_Y = zeros(3,9)

for angle = 1:3%
    angle_deg = (angle*30)-60;
    psi = deg2rad(angle_deg);
    
    for distance = 1:9%
        distance_to_gate = 1.8 - (distance/6);
        dist_(distance) = distance_to_gate;
        drone_pos = [cos(psi)*distance_to_gate sin(psi)*distance_to_gate]

        rel_1 = [drone_pos(1)-s_1(1) s_1(2)-drone_pos(2)];
        rel_2 = [drone_pos(1)-s_2(1) s_2(2)-drone_pos(2)];

        %2D projection:
        %rotate into image coordinate frame
        rot_2d =[cos(psi) -sin(psi);
                sin(psi)    cos(psi)];
         rot_1 = rot_2d*rel_1';
         rot_2 = rot_2d*rel_2';
        %normalize coordinates
        norm_1 = rot_1/rot_1(1);
        norm_2 = rot_2/rot_2(1);


        total_error_x = zeros(sample_size,1);
        total_error_y = zeros(sample_size,1);

        for n = 1:sample_size
        %2d camera projection to pixels with noise
        pix_noise = 10;
        psi_noise = 0;%deg2rad(10);
        f = 168;
        pix_1 = norm_1(2)*f + (rand(1,1)-0.5)*pix_noise;
        pix_2 = norm_2(2)*f + (rand(1,1)-0.5)*pix_noise;
        %convert to angles and rotate back
        alpha_1 = atan2(pix_1,f) - psi + (rand(1,1)-0.5)*psi_noise;
        alpha_2 = atan2(pix_2,f) - psi + (rand(1,1)-0.5)*psi_noise;
        %rotate back

        beta = deg2rad(90)-alpha_1;
        gamma = deg2rad(90)+alpha_2;

        r_1 = (sin(gamma)*gate_size)/sin(alpha_1-alpha_2);
        %r_2 = (sin(beta)*gate_size)/sin(alpha)

        x_h = cos(alpha_1)*r_1;
        y_h = gate_size/2-sin(alpha_1)*r_1;

        error = drone_pos - [x_h y_h];
        total_error_x(n) = error(1);
        total_error_y(n) = error(2);
        end
        
        RMSE_X(angle,distance) = sqrt(mean(total_error_x.^2))
        RMSE_Y(angle,distance) = sqrt(mean(total_error_y.^2))

    end
end

figure()
plot(dist_,RMSE_X(1,:),'-*')
hold on
plot(dist_,RMSE_X(2,:),'-+')
hold on
plot(dist_,RMSE_X(3,:),'-o')

grid on
legend('-30 deg','0 deg','30 deg')
xlabel('Distance to gate [M]') % x-axis label
ylabel('X RMSE [M]') % y-axis label
axis([0 2, 0 0.07])
x0=10;
y0=10;
width=250;
height=200;
set(gca,'FontSize',8)
set(gcf,'units','points','position',[x0,y0,width,height])
saveas(gcf,'../autosave/png/hist_RMSE_x.png')
saveas(gcf,'../autosave/eps/hist_RMSE_x','epsc')

figure()
plot(dist_,RMSE_Y(1,:),'-*')
hold on
plot(dist_,RMSE_Y(2,:),'-+')
hold on
plot(dist_,RMSE_Y(3,:),'-o')

grid on
legend('-30 deg','0 deg','30 deg')
xlabel('Distance to gate [M]') % x-axis label
ylabel('Y RMSE [M]') % y-axis label
axis([0 2, 0 0.07])
x0=10;
y0=10;
width=250;
height=200;
set(gca,'FontSize',8)
set(gcf,'units','points','position',[x0,y0,width,height])
saveas(gcf,'../autosave/png/hist_RMSE_y.png')
saveas(gcf,'../autosave/eps/hist_RMSE_y','epsc')