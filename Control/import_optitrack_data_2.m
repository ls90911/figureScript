function [OT] = import_optitrack_data_2(name,body_names)
%% Optitrack import file modified by Sihao
%   Coordinate definition:
%   G: Ground frame in Optical Track (Z black arrow, X left, Y up)
%   E: Earth frame. Cyberzoo or NED (X north, Y east, Z down)
%   O: OptiTrack body fixed frame (Z forward, X left, Y up)
%   B: Body fixed frame (X forward, Y right, Z down)
%   
%% Script Parameters
EarthFrame_NED = true;
% load calibration constants (B2IMU, O2B, hinge positions)
%load(['calibration data ' name_calib '.mat'])

posCG_B = [0 0 0]*1e-3; %m
posIMU_B = [0 0 0]*1e-3; %m
posCO_B = [0 0 0];%posCO_B*Rot_x(-pi);
%% Model Constants
d2r     = deg2rad(1);
r2d     = rad2deg(1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Import OptiTrack data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Nbodies=length(body_names);
[name '.csv']
openedFile = fopen([name '.csv'],'r');

% read the first 7 lines and save individiual cells delimited by comma
for i=1:7
    line=fgetl(openedFile);
    lines(i).string=regexp(line, ',', 'split');
end

% parse the general properties
data.formatVersion=str2double(lines(1).string(2));
data.fps=str2double(lines(1).string(8));
data.date=lines(1).string(10);
data.Nframes=str2double(lines(1).string(14));
data.filename=name;

if data.formatVersion~=1.21
    disp('Warning: this format version of OptiTrack csv file has not been tested, please check the results carefully')
end

% create index of all the columns
index=1:length(lines(4).string);

% load the rest of the data
data.CSVdata=csvread([name '.csv'],7,0);
data.frames=data.CSVdata(:,1);
data.time=data.CSVdata(:,2);

%% Compute OT information for each body
for n=1:Nbodies
    data.body(n).name=body_names(n);
    Nchar=length(char(body_names(n)));

    % for each body, find the relevant columns
    nameMatchFull=strcmp(lines(4).string,char(body_names(n)));
    if sum(nameMatchFull)==0
        disp(['Warning: Body with name "' char(body_names(n)) '" not found in the log.'])
            continue
    end

    nameMatchPartial=strncmp(lines(4).string,char(body_names(n)),Nchar);
    markerMatch=strcmp(lines(3).string,'Marker');
    rigidBodyMarkerMatch=strcmp(lines(3).string,'Rigid Body Marker');

    % body columns
    data.body(n).cBody=index(nameMatchFull);
    % rigid body (fitted) marker columns (4 columns per marker)
    data.body(n).cMarkersFitted=index((nameMatchPartial-nameMatchFull).*rigidBodyMarkerMatch==1);
    Nmarkers=length(data.body(n).cMarkersFitted)/4; % 4 columns per marker
    % raw marker columns (3 columns per marker)
    data.body(n).cMarkers=index((nameMatchPartial-nameMatchFull).*markerMatch==1);

    % set the lines when body was untracked (marker error==0) to NaN
    nanframes=data.frames(data.CSVdata(:,data.body(n).cBody(8))==0)+1;
    data.CSVdata(nanframes,index(nameMatchPartial==1))=NaN;

    % body position, orientation and mean marker error
    data.body(n).pos=data.CSVdata(:,data.body(n).cBody([5 6 7])); % OptiTrack z,x,y --> x,y,z
    data.body(n).quat=data.CSVdata(:,data.body(n).cBody(1:4));
    data.body(n).meanError=data.CSVdata(:,data.body(n).cBody(8));

    % marker positions
    j=0;
    for i=1:Nmarkers
        data.body(n).marker(i).posFitted=data.CSVdata(:,data.body(n).cMarkersFitted([4*i-1 4*i-3 4*i-2])); % OptiTrack z,x,y --> x,y,z
        data.body(n).marker(i).qual=data.CSVdata(:,data.body(n).cMarkersFitted(4*i));

        if sum(abs(data.body(n).marker(i).qual),'omitnan')==0 % the matching marker was not seen in the entire recording
            data.body(n).marker(i).pos=NaN(data.Nframes,3);
        else
            if ~isempty(data.body(n).cMarkers) % if no markers were exported, skip the following
               j=j+1; % increment only if the marker was seen
                data.body(n).marker(i).pos=data.CSVdata(:,data.body(n).cMarkers([3*j 3*j-2 3*j-1])); % OptiTrack z,x,y --> x,y,z
            end
        end
    end

    % Read optitrack position
    time_track     = data.time;
    posCO_G  = data.body(n).pos ;

    % OptiTrack orientation quaternions -  pitch, then yaw (opposite), then roll
    iprev=1;

    roll_G2O    = NaN(data.Nframes,1);
    pitch_G2O   = NaN(data.Nframes,1);
    yaw_G2O     = NaN(data.Nframes,1);

    qx=data.body(n).quat(:,1);
    qy=data.body(n).quat(:,2);
    qz=data.body(n).quat(:,3);
    qw=data.body(n).quat(:,4);

    % set to NaN if untracked
    for i=1:data.Nframes
        if isequal(posCO_G(i,:),[0 0 0])
            posCO_G(i,:) = NaN(1,3);
            qx(i)= NaN;
            qy(i)= NaN;
            qz(i)= NaN;
            qw(i)= NaN;
        end
    end

    for i=1:data.Nframes
        % rotation matrix from OptiTrack quaternion
        R = [1-2*(qy(i)^2+qz(i)^2)        2*(qx(i)*qy(i)-qz(i)*qw(i))  2*(qx(i)*qz(i)+qy(i)*qw(i))
             2*(qx(i)*qy(i)+qz(i)*qw(i))  1-2*(qx(i)^2+qz(i)^2)        2*(qy(i)*qz(i)-qx(i)*qw(i))
             2*(qx(i)*qz(i)-qy(i)*qw(i))  2*(qy(i)*qz(i)+qx(i)*qw(i))  1-2*(qx(i)^2+qy(i)^2)      ];

        % orientation from OptiTrack log (roll around OptiTrack Z, pitch around
        % OptiTrack X, Yaw around OptiTrack Y)
        roll_G2O(i,1)   = atan2d(R(2,1),R(2,2));
        pitch_G2O(i,1)  = atan2d(-R(2,3),real(sqrt(1-R(2,3)^2))); % real added to avoid complex numbers (most likely due to rounding errors)
        yaw_G2O(i,1)    = atan2d(R(1,3),R(3,3));

        % making yaw continuous
        if ~isnan(yaw_G2O(i,1))
            if i>1
                while abs(yaw_G2O(i,1)-yaw_G2O(iprev,1))>180
                    if yaw_G2O(i,1)-yaw_G2O(iprev,1)>180
                        yaw_G2O(i,1)    = yaw_G2O(i,1)-360;
                    elseif yaw_G2O(i,1)-yaw_G2O(iprev,1)<-180
                        yaw_G2O(i,1)    = yaw_G2O(i,1)+360;
                    end
                end
                iprev = i;
            end
        end
    end

    if EarthFrame_NED == true
        R_G2E = Rot_z(-(90-33)*d2r)*Rot_y(0)*Rot_x(-90*d2r);
        yaw_E2B_bias      = -yaw_G2O + 33;        
    else
        R_G2E = Rot_z(-(90)*d2r)*Rot_y(0)*Rot_x(-90*d2r);
        yaw_E2B_bias      = -yaw_G2O;         
    end
    posCO_E = posCO_G*R_G2E'; 
    roll_E2B_bias     = roll_G2O;
    pitch_E2B_bias    = -pitch_G2O;    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Compute roll, pitch and yaw of body frame
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    roll_E2B    =NaN(data.Nframes,1);
    pitch_E2B   =NaN(data.Nframes,1);
    yaw_E2B     =NaN(data.Nframes,1);

    % Rotation matrix from OT frame to Body frame by Calibration
    % Note R_Cali != R_O2B;
    % Define a new frame N at the origin of OT frame, and Xn = Zo, Yn = -Xo, Zn = -Yo
    % R_Cali = R_N2B;
    R_Cali = [1 0 0;0 1 0;0 0 1];

    iprev=1;
    for i=1:data.Nframes

        R_B2E = Rot_z(yaw_E2B_bias(i)*d2r)*Rot_y(pitch_E2B_bias(i)*d2r)* Rot_x(roll_E2B_bias(i)*d2r);

        roll_E2B(i,1)   = atan2d(R_B2E(3,2),R_B2E(3,3));
        pitch_E2B(i,1)  = atan2d(-R_B2E(3,1),real(sqrt(1-R_B2E(3,1)^2))); % real added to avoid complex numbers (most likely due to rounding errors)
        yaw_E2B(i,1)    = atan2d(R_B2E(2,1),R_B2E(1,1));       

        % making yaw continuous
        if ~isnan(yaw_E2B(i,1))
            if i>1
                while abs(yaw_E2B(i,1)-yaw_E2B(iprev,1))>180
                    if yaw_E2B(i,1)-yaw_E2B(iprev,1)>180
                        yaw_E2B(i,1)=yaw_E2B(i,1)-360;
                    elseif yaw_E2B(i,1)-yaw_E2B(iprev,1)<-180
                        yaw_E2B(i,1)=yaw_E2B(i,1)+360;
                    end
                end
                iprev=i;
            end
        end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Position, velocity and acceleration at IMU and CG from OptiTrack data
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % position of CG and IMU in earth frame
    posCG_E     = NaN(data.Nframes,3);
    posIMU_E    = NaN(data.Nframes,3);
    for i=1:data.Nframes
        Rot_B2E         = Rot_z(d2r*yaw_E2B(i)) * Rot_y(d2r*pitch_E2B(i)) * Rot_x(d2r*roll_E2B(i));
        posCG_E(i,:)    = posCO_E(i,:) + (Rot_B2E*(posCG_B-posCO_B)')';
        posIMU_E(i,:)   = posCO_E(i,:) + (Rot_B2E*(posIMU_B-posCO_B)')';
    end

    % velocities from optitrack positions
    velCO_E     = derivative(posCO_E,time_track);
    velCG_E     = derivative(posCG_E,time_track);
    velIMU_E    = derivative(posIMU_E,time_track);

    % accelerations from optitrack
    accCO_E = derivative(velCO_E,time_track);
    accCG_E = derivative(velCG_E,time_track);
    accIMU_E = derivative(velIMU_E,time_track);

    % transformation to DelFly body axes and LD (aero) axes
    velIMU_B    = NaN(data.Nframes,3);
    velCG_B     = NaN(data.Nframes,3);
    velCO_B     = NaN(data.Nframes,3);

    accIMU_B    = NaN(data.Nframes,3);
    accCG_B     = NaN(data.Nframes,3);
    accCO_B     = NaN(data.Nframes,3);

    for i = 1:data.Nframes
        Rot_E2B = Rot_x(-roll_E2B(i)*d2r)*Rot_y(-pitch_E2B(i)*d2r)*Rot_z(-yaw_E2B(i)*d2r);

        velIMU_B(i,:)   = (Rot_E2B*velIMU_E(i,:)')';
        velCG_B(i,:)    = (Rot_E2B*velCG_E(i,:)')';
        velCO_B(i,:)    = (Rot_E2B*velCO_E(i,:)')';

        accIMU_B(i,:)   = (Rot_E2B*accIMU_E(i,:)')';
        accCG_B(i,:)    = (Rot_E2B*accCG_E(i,:)')';
        accCO_B(i,:)    = (Rot_E2B*accCO_E(i,:)')';

    end

    %% angular rates and accelerations

    rollt_E2B   = derivative(roll_E2B*d2r,time_track);
    pitcht_E2B  = derivative(pitch_E2B*d2r,time_track);
    yawt_E2B    = derivative(yaw_E2B*d2r,time_track);

    om_B_track = NaN(data.Nframes,3);
    for i = 1:data.Nframes
        AA = [1  0                 -sind(pitch_E2B(i));
            0  cosd(roll_E2B(i))  sind(roll_E2B(i))*cosd(pitch_E2B(i));
            0 -sind(roll_E2B(i))  cosd(roll_E2B(i))*cosd(pitch_E2B(i)) ];

        om_B_track(i,:) = (AA*[rollt_E2B(i);pitcht_E2B(i);yawt_E2B(i)])';
    end

    alph_B_track = derivative(om_B_track,time_track);

    % another way of computing the acceleration at CG
    % used to verify that the code used for IMU works
    accCG_B2 = NaN(data.Nframes,3);
    for i = 1:length(accCO_E(:,1))
        OM = [ 0               -om_B_track(i,3)  om_B_track(i,2)
             om_B_track(i,3)  0               -om_B_track(i,1)
            -om_B_track(i,2)  om_B_track(i,1)  0              ];
        
        ALP = [0                 -alph_B_track(i,3)  alph_B_track(i,2)
             alph_B_track(i,3)  0                 -alph_B_track(i,1)
            -alph_B_track(i,2)  alph_B_track(i,1)  0                ];
        
        accCG_B2(i,:) = accCO_B(i,:)+((OM^2+ALP)*(posCG_B-posCO_B)')';
    end
    
    % % low pass filtering
    % % replace all NaNs with zero before filtering
    % om_B_track0 = om_B_track;
    % om_B_track0(isnan(om_B_track0)) = 0;
    % % [btrack,atrack] = butter(4,2*Fcut/fps); % the argument is normalized frequency in pi*rad/sample, thus it needs to be multiplied by 2 (2*pi rad = 1 cycle)
    % % omxdf_B_track = filtfilt(btrack,atrack,om_B_track0(:,1))*r2d;
    % % omydf_B_track = filtfilt(btrack,atrack,om_B_track0(:,2))*r2d;
    % % omzdf_B_track = filtfilt(btrack,atrack,om_B_track0(:,3))*r2d;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Save optitrack data and resample to IMU frequency
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % savetime = time_IMU;

    OT.att_G2O     = [roll_G2O pitch_G2O yaw_G2O];
    % Mean marker error of the rigid body
    OT.mean_error  = data.body(n).meanError;
    OT.posCO_G     = posCO_G;
    OT.posCG_E     = posCG_E;
    OT.X           = posCG_E(:,1);
    OT.Y           = posCG_E(:,2);
    OT.Z           = posCG_E(:,3);
    % Attitude in B-frame: roll_E2B, pitch_E2B, yaw_E2B; (calc)
    OT.PHI         = roll_E2B;
    OT.THETA       = pitch_E2B;
    OT.PSI         = yaw_E2B;
    % INDIRECT (NB all unfiltered)
    % Original velocity in E-frame (num. diff.): velCO_E  (calc)
    OT.vel_E       = velCO_E;
    % CG velocity in E-frame (num. diff.): velCG_E (calc)
    OT.velCG_E    = velCG_E;
    OT.VX          = velCG_E(:,1);
    OT.VY          = velCG_E(:,2); 
    OT.VZ          = velCG_E(:,3); 
    % Original acceleration in E-frame (num. diff.): accCO_E (calc)
    OT.acc_E       = accCO_E;
    % CG acceleration in E-frame(num. diff.): accCG_E (calc)
    OT.accCG_E     = accCG_E;
    % Velocity in B-frame: velCO_B (calc)
    OT.vel_B       = velCO_B;
    % CG velocity in B-frame: velCG_B (calc)
    OT.velCG_B     = velCG_B;
    OT.U           = velCG_B(:,1);
    OT.V           = velCG_B(:,2);
    OT.W           = velCG_B(:,3);
    % Acceleration in B-frame: accCO_B (calc)
    OT.acc_B       = accCO_B;
    % CG acceleration in B-frame: accCG_B (calc)
    OT.accCG_B     = accCG_B;
    OT.AX          = accCG_B(:,1);
    OT.AY          = accCG_B(:,2);
    OT.AZ          = accCG_B(:,3);
    % Attitude derivatives (num. diff) in B frame: rollt_B2E, pitcht_B2E,
    OT.PHIDOT_B    = rollt_E2B;
    OT.THETADOT_B  = pitcht_E2B;
    OT.PSIDOT_B    = yawt_E2B;
    % Angular rates (from attitude) in B frame: om_B_track (or om_B_track0 without NaN)
    OT.OMEGA_B     = om_B_track; % unfiltered!
    OT.P           = om_B_track(:,1);
    OT.Q           = om_B_track(:,2);
    OT.R           = om_B_track(:,3);
    % Angular accelerations (from attitude) in B frame: alph_B_track
    OT.angacc_B    = alph_B_track;
    OT.TIME        = time_track;

    OT.QDOT = derivative(OT.Q,OT.TIME);
    OT.UDOT = derivative(OT.U,OT.TIME);
    OT.WDOT = derivative(OT.W,OT.TIME);
end
% %%% resampled data
% OT = struct;
% allnames = fieldnames(OT_orig);
% for i = 1:length(fieldnames(OT_orig))
%     ct_orig = OT_orig.(allnames{i});
%     ct_orig = interpNan(ct_orig);
%     %ct_orig(isnan(ct_orig)==1)=0;
%     OT.(allnames{i}) = interp1(time_track,ct_orig,time_IMU,'spline');
% end
% OT.time = time_IMU;
end
