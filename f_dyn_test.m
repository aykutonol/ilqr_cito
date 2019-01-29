function dx = f_dyn_test(xk,uk,dt,acon,vd,ec,eo,mo,Jo)
% returns the nonlinear dynamics given robot and object states
%% Parse inputs
qpos = xk(1:4);     qvel = xk(5:8);
opos = xk(9:11);    ovel = xk(12:14);
gam0 = xk(15:18);
%% change
tau  = uk(1:4,:);   kcon = uk(5:8,:);
%% change
%% End-effector position
pe(1,1) = 2/5 - (3*sin(qpos(1) + qpos(2) + qpos(3) + qpos(4)))/10 - (3*sin(qpos(1) + qpos(2)))/10 - (3*sin(qpos(1)))/10 - (3*sin(qpos(1) + qpos(2) + qpos(3)))/10;
pe(2,1) = (3*cos(qpos(1) + qpos(2) + qpos(3)))/10 + (3*cos(qpos(1) + qpos(2) + qpos(3) + qpos(4)))/10 + (3*cos(qpos(1) + qpos(2)))/10 + (3*cos(qpos(1)))/10;
%% Contact model
pv(:,1:4) = opos(1:2)*ones(1,4) + f_rot2d(opos(3))*vd*eo/2; pv(:,5) = pv(:,1);
pc(:,1:4) = opos(1:2)*ones(1,4) + f_rot2d(opos(3))*ec*eo/2;
nd = f_rot2d(opos(3))*ec;
for i = 1:4
%     phi(i,1) = p_poly_dist(pe(1),pe(2),pv(1,i:i+1),pv(2,i:i+1));
    phi_n(i) = dot(pc(:,i)-pe,-nd(:,i));
    phi_e = norm(pc(:,i)-pe,2);
    zeta(i) = tanh(20*phi_e);
    phi(i,1) = zeta(i)*phi_e + (1-zeta(i))*phi_n(i);
end
gamma = kcon.*exp(-acon*phi);
% penetration
% [phi_min,~, ~, ~, edge_ind] = p_poly_dist_ext(pe(1),pe(2),pv(1,:),pv(2,:));
in = sum(sign(phi_n))==-4;
if in
    [phi_n_min,edge_ind] = max(phi_n);
end
% total wrench
hc = [0;0]; hp = [0;0];
for i = 1:4
    ca = mod(opos(3),2*pi) + i*pi/2;
    hc = hc+gamma(i)*[cos(ca);sin(ca)];
    if in && i == edge_ind
        hp = -1e4*phi_n_min*[cos(ca);sin(ca)];
    end
end
%% Object dynamics
L = pe - opos(1:2);                               % vector between contact point and CoM
% L = zeros(2,1);
ho = [eye(2) zeros(2,1);f_skew2d(L) 1]*[hc+hp;0]; % object wrench
f_fr = mo*9.8*0.75;
for i = 1:3
    ho_fr(i,1) = -tanh(100*ovel(i))*f_fr;
end
ho = ho + ho_fr;
oacc = [ho(1:2)/mo;ho(3)/Jo];
%% Robot dynamics
[Minv,C,Jt] = f_evalMinvCJt(qpos,qvel);
qacc = Minv*(tau - Jt'*(hc+hp) - C*qvel);
dgamma = (gamma-gam0)/dt;
%% Change of states
dx = [qvel;qacc;ovel;oacc;dgamma];