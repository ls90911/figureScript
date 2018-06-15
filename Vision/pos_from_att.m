function [pos_estimate] = pos_from_att(gate_points,cam_pos,img_points_x,img_points_y,phi_n, theta_n, psi_n)

%line parametarization p = a+ tn

%matrix of 2d point position vectors
a = zeros(3,3);


f = 168;

R_mat_n = C_b_n( phi_n, theta_n, psi_n );

image_vectors = zeros(3,4);
for n = 1:4
     image_vectors(:,n) = R_mat_n*vec_from_image_point_2( img_points_x(n), img_points_y(n),f );
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
