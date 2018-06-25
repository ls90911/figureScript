function [] = plotCube(origin,xlength,ylength,height,theta,figNum,color,...
    transparent)
% This function is used to plot cube in 3d plot

p1 = [0 0 0];
p2 = [xlength*cos(3*pi/2+theta) xlength*sin(3*pi/2+theta) 0];;
p3 = [ylength*cos(theta) ylength*sin(theta) 0] + p2;
p4 = [ylength*cos(theta) ylength*sin(theta) 0];
p5 = p1 + [0 0 height];
p6 = p2 + [0 0 height];
p7 = p3 + [0 0 height];
p8 = p4 + [0 0 height];

vertex = [p1+origin;p2+origin;p3+origin;p4+origin;p5+origin;p6+origin;p7+origin;p8+origin];
face = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];

figure(figNum)
p = patch('Vertices',vertex,'Faces',face);
p.FaceColor = color;
p.FaceAlpha = transparent;
p.EdgeColor = color;
%p.EdgeAlpha = transparent;
axis equal;


end

