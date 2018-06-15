function y = find_R2(x,z) 
% Compute R2 to evaluate model accuracy
% x : model output, 1 demension vector
% z : measurement, 1 demension vector
% Sihao Sun 21-12-2016

y = 1-sum((z-x).^2)/sum((z-mean(z)).^2);
end
