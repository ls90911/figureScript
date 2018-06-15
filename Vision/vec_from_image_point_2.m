function [ image_vectors ] = vec_from_image_point_2( u, v, f )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
  
    %3x3 empty matrix
    %image_vectors = zeros(3,3);
    
    x = 1;%
    y = u/f;
    z = v/f;
    vec = [x y z];
    image_vectors = vec';
    
end

