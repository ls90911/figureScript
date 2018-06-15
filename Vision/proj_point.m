function [ u, v ] = proj_point( gate_point,intr, R, cam_pos )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    point_3d = R*(gate_point-cam_pos');

    hom_coord = [point_3d(2)/point_3d(1);
                 point_3d(3)/point_3d(1);
                 1];

    res = intr*hom_coord;
    
    u = res(1);
    v = res(2);%still up side down due to ned convention

end

