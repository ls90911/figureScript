function Mat = Kin_att( theta, phi )
%KIN_ATT Summary of this function goes here
%   Detailed explanation goes here

Mat = [1 tan(theta)*sin(phi) tan(theta)*cos(phi);...
    0 cos(phi) -sin(phi);...
0 sin(phi)/cos(theta) cos(phi)/cos(theta)];

end

