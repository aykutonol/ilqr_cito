function pl = f_link_pos(q)
% Returns links' end positions given joint positions
pl(1,1) =  2/5 - (3*sin(q(1)))/10;
pl(1,2) = (3*cos(q(1)))/10;
pl(2,1) =  2/5 - (3*sin(q(1)))/10 - (3*sin(q(1) + q(2)))/10;
pl(2,2) =  (3*cos(q(1) + q(2)))/10 + (3*cos(q(1)))/10;
pl(3,1) =  2/5 - (3*sin(q(1) + q(2)))/10 - (3*sin(q(1)))/10 - (3*sin(q(1) + q(2) + q(3)))/10;
pl(3,2) =  (3*cos(q(1) + q(2) + q(3)))/10 + (3*cos(q(1) + q(2)))/10 + (3*cos(q(1)))/10;
pl(4,1) = 2/5 - (3*sin(q(1) + q(2) + q(3) + q(4)))/10 - (3*sin(q(1) + q(2)))/10 - (3*sin(q(1)))/10 - (3*sin(q(1) + q(2) + q(3)))/10;
pl(4,2) = (3*cos(q(1) + q(2) + q(3)))/10 + (3*cos(q(1) + q(2) + q(3) + q(4)))/10 + (3*cos(q(1) + q(2)))/10 + (3*cos(q(1)))/10;