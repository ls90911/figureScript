close all
clear
clc

load('basementData');
figure(1)
start_plotting = 2;
    [end_ , ~]= size(Flight_pos{1});
    end_ = 17000;
    plot3(Flight_pos{1}(start_plotting:end_,1),-Flight_pos{1}(start_plotting:end_,2),Flight_pos{1}(start_plotting:end_,3))
    hold on
    
    start_plotting = 2;
    [end_ , ~]= size(Flight_pos{2});
    end_ = 18250;
    plot3(Flight_pos{2}(start_plotting:end_,1),-Flight_pos{2}(start_plotting:end_,2),Flight_pos{2}(start_plotting:end_,3))
    hold on
    
    start_plotting = 2;
    [end_ , ~]= size(Flight_pos{3});
    end_ = 15600;
    plot3(Flight_pos{3}(start_plotting:end_,1),-Flight_pos{3}(start_plotting:end_,2),Flight_pos{3}(start_plotting:end_,3))
    hold on
    
    for i = 1:5
        %gate at origin
        g_1_x = 0.0;
        g_1_y = 0.0;
        test_3d_plot = [0+g_1_x 0+g_1_y 0; 0+g_1_x 0+g_1_y 1;0+g_1_x -0.5+g_1_y 1;0+g_1_x -0.5+g_1_y 2;0+g_1_x 0.5+g_1_y 2;0+g_1_x 0.5+g_1_y 1;0+g_1_x 0+g_1_y 1];
        gate_angle = deg2rad(gate_psi(i));
        rot_2d = [cos(gate_angle) -sin(gate_angle);
            sin(gate_angle) cos(gate_angle)];
        temp = rot_2d*test_3d_plot(1:7,1:2)';
        test_3d_plot(1:7,1:2) = temp';
        if i == 5
            gate = plot3(test_3d_plot(:,1)+gate_x(i),-(test_3d_plot(:,2)+gate_y(i)),test_3d_plot(:,3),'Linewidth',3,'Color',[255,0,0]/255)
        else
            gate = plot3(test_3d_plot(:,1)+gate_x(i),-(test_3d_plot(:,2)+gate_y(i)),test_3d_plot(:,3),'Linewidth',3,'Color',[255,69,0]/255)
        end
        gate.Color(4) = 0.9;
        hold on
    end
    
    caseInBasement = [8,1.5,0,1.1,7.5,2.2,pi/2;...
        4.85,1.15,0,0.8,1.4,1.8,pi/2;...
        5,3.2,0,1,1.3,0.5,pi/2;...
        5,7.7,0,1.1,3.6,2.2,pi/2;...
        5,14,0,1.1,2.4,2.2,pi/2;...
        3.6,13.7,0,0.3,0.3,2.2,pi/2;...
        3.6,9.9,0,0.3,0.3,2.2,pi/2;...
        3.6,6.1,0,0.3,0.3,2.2,pi/2;...
        3.6,2.3,0,0.3,0.3,2.2,pi/2;...
        3.6,7.1,0,0.5,2,0.5,pi/2;...
        -1,6.1,0,0.3,0.3,2.2,pi/2;...
        -6,6.1,0,0.3,0.3,2.2,pi/2;...
         -6,2.1,0,0.3,0.3,2.2,pi/2;...
         -6.3,3,0,1.1,2.4,2.2,pi/2;...
         -1,10.1,0,0.3,0.3,2.2,pi/2;...
         -0.5,5.3,0,2,1.2,1,pi/2;...
          -0.4,11.1,0,1.1,3,1.8,pi/2;...
          -6,10.1,0,0.3,0.3,2.2,pi/2;...
           -6.3,7,0,1.1,2.4,2.2,pi/2;...
           -1,2.3,0,0.3,0.3,2.2,pi/2;...
           -1,3.6,0,3.5,1.15,2.2,pi/2;...
           0,2.3,0,2,1,1,pi/2;...
            5,5,0,1,1,0.5,pi/2;...
        ];
    figNum = 1;
    for i = 1:size(caseInBasement,1)
        plotCube(caseInBasement(i,1:3),caseInBasement(i,4),caseInBasement(i,5),caseInBasement(i,6),caseInBasement(i,7),figNum,[0.7,0.7,0.7],0.3);
    end
    
    
    axis equal
    grid on
    temp = 1;