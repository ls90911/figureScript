function [pos_estimate] = pos_from_att_ned_20(gate_points,cam_pos,img_points_x,img_points_y,phi_n, theta_n, psi_n)

%line parametarization p = a+ tn

%matrix of 2d point position vectors
a = zeros(3,3);

f = 168;

R_mat_n = C_b_n( phi_n, theta_n, psi_n );

R_mat_20 = C_b_n( 0, deg2rad(-20), 0 );

plot_on = 0;

 attitude = [rad2deg(phi_n) rad2deg(theta_n) rad2deg(psi_n)];

image_vectors = zeros(3,4);
for n = 1:4
    img_vec = vec_from_image_point_2( img_points_x(n), img_points_y(n),f );
    vec_body = R_mat_20'*img_vec;
     image_vectors(:,n) = R_mat_n'*vec_body;%
     temp = image_vectors(3,n);
     image_vectors(3,n) = -temp;
     if(plot_on)
      waitforbuttonpress
     end
end

%matrix of direction vectors
n = zeros(3,4);
n(:,1) = image_vectors(:,1);%
n(:,1) = n(:,1)/norm(n(:,1),2);
n(:,2) = image_vectors(:,2);%
n(:,2) = n(:,2)/norm(n(:,2),2);
n(:,3) = image_vectors(:,3);%
n(:,3) = n(:,3)/norm(n(:,3),2);
n(:,4) = image_vectors(:,4);%
n(:,4) = n(:,4)/norm(n(:,4),2);

a(:,1) = gate_points(:,1);
a(:,2) = gate_points(:,2);
a(:,3) = gate_points(:,3);
a(:,4) = gate_points(:,4);

if(plot_on)
t = linspace(-200,200);

for i = 1:4
X = a(1,i)+ t.*n(1,i);
Y = a(2,i)+ t.*n(2,i);
Z = a(3,i)+ t.*n(3,i);
plot3(X,Y,Z)
hold on
plot3(a(1,:),a(2,:),a(3,:))
plot3(a(1,:),a(2,:),a(3,:),'*')
hold on
end
grid on
axis([0 5, -1 2, -2 2])
end
%calculate intercection point
R = zeros(3,3);
q = zeros(3,1);

for i = 1:4
    R = R + (eye(3,3)-n(:,i)*n(:,i)');
    q = q + (eye(3,3)-n(:,i)*n(:,i)')*a(:,i);
end

P = R\q;
%plot3(P(1),P(2),P(3),'*')
pos_estimate = P;
if(plot_on)
pos_estimate = P;
end
