function [ gate_points ] = Calc_gate_points_order( gate_pos )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

P1 = gate_pos + [0 -0.5 -0.5]; %left up
P2 = gate_pos + [0  0.5 -0.5];%right up
P3 = gate_pos + [0  0.5  0.5];%right down 
P4 = gate_pos + [0 -0.5  0.5];%left down
P5 = gate_pos + [0  0    0.5];%middle bottom
P6 = gate_pos + [0  0    1.4];

gate_points = [P1' P2' P3' P4' P5' P6'];


end

