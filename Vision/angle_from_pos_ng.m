function [pos_a, phi_est, theta_est, psi_est, search_points_x ,search_points_y] = angle_from_pos_ng(gate_position,cam_position,sim_points_x,sim_points_y,phi_gate,theta_gate,psi_gate)

%not use ned
gate_pos = gate_position;
cam_pos  = cam_position;

cam_gd = sqrt((gate_pos(1)-cam_pos(1))^2+(gate_pos(2)-cam_pos(2))^2)
cam_a = atan((gate_pos(2)-cam_pos(2))/(gate_pos(1)-cam_pos(1)))

genome0 = [phi_gate theta_gate psi_gate cam_a];

genome_length = length(genome0);

ga_step = deg2rad(0.5);%

n_individuals = 100;
Population = repmat(genome0, [n_individuals, 1]) + ga_step * rand(n_individuals, genome_length) - (ga_step/2);
n_generations = 500;%
best_fit = 100000;
best_genome = Population(1, :);

for g = 1:n_generations

fits = zeros(1,n_individuals);
for i = 1:n_individuals
    
    phi_g =  Population(i, 1);%%positive rotation left ccw
    theta_g = Population(i, 2);%;%positive down
    psi_g = Population(i, 3);%;%positive to the left
    cam_a = Population(i, 4);%
  
    [search_points_x, search_points_y, pos_a] = Gate_angle_search_ng(gate_position,cam_pos(3),cam_gd,cam_a,phi_g,theta_g,psi_g);
    
    phi_error = rad2deg(phi_g);
    theta_error = rad2deg(theta_g);
    psi_error = rad2deg(psi_g);
    
    pix_error = zeros(4,1);
    for n = 1:4
     pix_error(n) = sqrt((sim_points_x(n)-search_points_x(n)+153.2)^2+(sim_points_y(n)-search_points_y(n)+32)^2);
    end
    
    fits(i) = sum(pix_error);%

end

    [v, ind] = min(fits);
    if(v < best_fit)
        best_genome = Population(ind, :);
        best_fit = v;
    end
    if(g < n_generations)
        Population = repmat(Population(ind, :), [n_individuals, 1]);%original
        Mutation = ga_step * rand(n_individuals, genome_length) - (ga_step/2);
        Population = Population + Mutation;
    end

    phi_g =  best_genome(1);%positive rotation left ccw
    theta_g = best_genome(2);%positive down
    psi_g = best_genome(3);%positive to the left
    cam_a = best_genome(4);

end
best_fit
    phi_est = phi_g;
    theta_est = theta_g;
    psi_est = psi_g;

pos_a(3) = -pos_a(3);
