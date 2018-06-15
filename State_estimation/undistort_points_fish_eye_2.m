
function [corX, corY] = undistort_points_fish_eye_2(image_points_x,image_points_y)


x_p = 153.2;%
y_p = 32;%
f = 168;

k = 1.150;

%distortion with fisheye model
%from now on stay in coord frame of last figure
%plot(0,0,'*')
corX = zeros(4,1);
corY = zeros(4,1);
for i = 1:4
    
    %convert to poolar coordinates
    x_mid = image_points_x(i)-x_p;
    y_mid = image_points_y(i)-y_p;
    
    r = sqrt((x_mid^2)+(y_mid^2));
    theta = atan2(y_mid,x_mid);
    
    %corR = f * tan( asin( sin( atan( r / f ) ) * k ) );
   R = f*tan(asin(sin( atan(r/f))*k));
   corX(i) = R * cos(theta)+x_p;
   corY(i) = R * sin(theta)+y_p;
    
    %plot(x_mid,y_mid,'o')
end
% figure();
% plot(x_mid,y_mid,'o')
% plot(corX,corY,'+')
% axis([0 315, 0 160]) 






