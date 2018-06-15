
function [corX, corY] = undistort_points_fish_eye_4a(image_points_x,image_points_y,cameraParameters)


    
   temp = undistortPoints([image_points_y' 315-image_points_x'],cameraParameters);
    
   corX = -temp(:,2)+315;
   corY = temp(:,1);
   
 






