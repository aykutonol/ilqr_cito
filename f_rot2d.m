function R = f_rot2d(theta)
% returns 2D rotation matrix for theta [rad]
n = length(theta);
R = [];
for i = 1:n
    R = [R; [cos(theta(i)) -sin(theta(i)); sin(theta(i)) cos(theta(i))]];
end