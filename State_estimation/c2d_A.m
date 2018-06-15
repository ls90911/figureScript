function [ A_d ] = c2d_A( A,dt )
%Continous to discrete for EKF
%can be accuratly expressed by truncated series expansion, because jacobian is of upper triangular form 

   s = size(A,1);

    A_d = eye(s) + A*dt + (1/2)*A*A*dt;

end

