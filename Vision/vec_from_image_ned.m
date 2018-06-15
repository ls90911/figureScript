function [ image_vectors ] = vec_from_image_ned( u, v, f )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
  
    %3x3 empty matrix
    image_vectors = zeros(3,3);
    
    for n = 1:3
    x = f;%
    y = u(n);
    z = -v(n);
    vec = [x y z];
    vec = vec/norm(vec);
    image_vectors(:,n) = vec';
    end
end

