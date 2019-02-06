clear; clc; %close all;
set(groot,'defaultLineLineWidth',1,'defaultAxesGridAlpha',.3,...
          'defaultAxesGridColor','k','defaultAxesGridLineStyle',':');
addpath('ode');
%% Initialiation
opos0 = [.1;.6;0];          % [m,m,rad] initial object position
oposd = [.1;.5];            % [m,m]     desired object position
% parameters
ilqr_params;
% weights
w = [1e4; 1e-3; 0; 1e-4];
% save flag
saveFlag = 0;
%% Initial configuration and control trajectory
qpos0 = [-pi/3; pi/4; pi/4; pi/2.3];
qvel0 = [0;0;0;0];  ovel = [0;0;0];
epos0(1,1) = 2/5 - (3*sin(qpos0(1) + qpos0(2) + qpos0(3) + qpos0(4)))/10 - (3*sin(qpos0(1) + qpos0(2)))/10 - (3*sin(qpos0(1)))/10 - (3*sin(qpos0(1) + qpos0(2) + qpos0(3)))/10;
epos0(2,1) = (3*cos(qpos0(1) + qpos0(2) + qpos0(3)))/10 + (3*cos(qpos0(1) + qpos0(2) + qpos0(3) + qpos0(4)))/10 + (3*cos(qpos0(1) + qpos0(2)))/10 + (3*cos(qpos0(1)))/10;
pc0(:,1:4) = opos0(1:2)*ones(1,4) + f_rot2d(opos0(3))*ec*eo/2;
nd0 = f_rot2d(opos0(3))*ec;
pv(:,1:4) = opos0(1:2)*ones(1,4) + f_rot2d(opos0(3))*vd*eo/2; pv(:,5) = pv(:,1);
for i = 1:ncc
    phi_n0(i) = dot(pc0(:,i)-epos0,-nd0(:,i));
    phi_e0 = norm(pc0(:,i)-epos0,2);
    zeta0(i) = tanh(20*phi_e0);
    phi0(i,1) = zeta0(i)*phi_e0 + (1-zeta0(i))*phi_n0(i);
end
x0(:,1) = [qpos0;qvel0;opos0;ovel;zeros(ncc,1)];
% initial controls
tau = 0*ones(ndof,1);  kcon = kcon0*ones(ncc,1);
U = [tau;kcon]*ones(1,ncts);
% bounds on controls
lb = [-tau_lim*ones(size(tau)); zeros(size(kcon))];
ub = [ tau_lim*ones(size(tau)); kcon0*ones(size(kcon))];
% dimensions of the state and control vectors
global n m
n = size(x0,1);             % [-] number of state variables
m = size(U,1);              % [-] number of control variables
%% Iterative Linear Quadratic Regulator
% Parameters
lambda = 1; dlambda = 1; lambdaFactor = 1.6; lambdaMax = 1e10; lambdaMin = 1e-6;
Alpha = 10.^linspace(0,-3,11); zMin = 0; tolGrad = 1e-4; tolFun = 1e-6; maxIter = 100;
% Main loop
diffDone = 0;
t_ILQR = tic;
for iter = 1:maxIter
    trace(iter).iter = iter;
    % update the initial gamma values
    gamma0 = U(5:8,1).*exp(-acon*phi0);
    x0(15:18,1) = gamma0;
    %% Differentiation
    if ~diffDone
        t_diff = tic;
        [X,c,fx,fu,cx,cu,cxx,cxu,cuu] = f_diff_dyncst(x0,[U, nan(m,1)],oposd,w,h);
        trace(iter).timingDiff = toc(t_diff);
        diffDone = 1;
    end
    %% Backward pass
    bwdPassDone = 0;
    t_bwd_pass = tic;
    while ~bwdPassDone
        [diverge,kff,Kfb,Vx,Vxx,dV] = f_bwd_pass(fx,fu,cx,cu,cxx,cxu,cuu,lambda,m,n,ncts,ndts,lb,ub,U);
        % Check for divergence
        if diverge
            fprintf('\tWARNING: Cholesky failed at timestep %d.\n',diverge);
            dlambda = max(dlambda*lambdaFactor, lambdaFactor);
            lambda  = max(lambda*dlambda, lambdaMin);
            if lambda > lambdaMax, break; end
            continue
        end
        bwdPassDone = 1;
    end
    trace(iter).timingBwdPass = toc(t_bwd_pass);
    % Check for small gradient
    gradNorm = mean(max(abs(kff)./(abs(U)+1),[],1));
    trace(iter).gradNorm = gradNorm;
    if gradNorm < tolGrad && lambda < 1e-5
        dlambda = min(dlambda/lambdaFactor, 1/lambdaFactor);
        lambda  = lambda*dlambda*(lambda>lambdaMin);
        fprintf('\nSUCCESS: gradient norm < tolGrad\n');
        break;
    end
    %% Line search
    fwdPassDone = 0;
    if bwdPassDone
        t_fwd = tic;
        na = length(Alpha);
        for a = 1:na
            alpha = Alpha(a);
            [Xnew,Unew,cnew] = f_fwd_pass(x0,X,U,oposd,w,kff,Kfb,alpha);
            dcost    = c(end) - cnew(end);
            expected = -alpha*(dV(1) + alpha*dV(2));
            if expected > 0
                z = dcost/expected;
            else
                z = sign(dcost);
                warning('non-positive expected reduction: should not occur');
            end
            if z > zMin
                fwdPassDone = 1;
                break;
            end
        end
        if ~fwdPassDone
            alpha = nan;
        end
        trace(iter).timingFwdPass = toc(t_fwd);
    end
    %% Accept or reject changes & show results
    % Print the header
    if mod(iter,10) == 1
        fprintf('%-12s','iteration','cost','reduction','expected','gradient','log10(lambda)');
        fprintf('\n');
    end
    % Accept changes
    if fwdPassDone
        % Print iteration details
        fprintf('%-12d%-12.6g%-12.3g%-12.3g%-12.3g%-12.1f\n', ...
                iter, c(end), dcost, expected, gradNorm, log10(lambda));
        % Decrease lambda
        dlambda = min(dlambda/lambdaFactor, 1/lambdaFactor);
        lambda  = lambda*dlambda*(lambda>lambdaMin);
        % Accept changes
        U = Unew;
        X = Xnew;
        c = cnew;
        diffDone = 0;
        % Check stopping criteria
        if dcost < tolFun
            fprintf('\nSUCCESS: cost change < tolFun\n');
            break;
        end
    % Reject changes
    else
        % Increase lambda
        dlambda = max(dlambda*lambdaFactor, lambdaFactor);
        lambda  = max(lambda*dlambda, lambdaMin);
        % Print status
        fprintf('%-12d%-12s%-12.3g%-12.3g%-12.3g%-12.1f\n', ...
                iter,'NO STEP', dcost, expected, gradNorm, log10(lambda)); 
        % Check stopping criteria
        if lambda > lambdaMax
            fprintf('\nEXIT: lambda > lambdaMax\n');
            break;
        end
    end
    % Update trace
    trace(iter).lambda      = lambda;
    trace(iter).dlambda     = dlambda;
    trace(iter).alpha       = alpha;
    trace(iter).improvement = dcost;
    trace(iter).cost        = c(end);
    trace(iter).rdc_ratio   = z;
    % velocity cost
    qvel = X(5:8,:); ovel = X(12:14,:);
    trace(iter).cvel = w(2)*(qvel(:)'*qvel(:)+ovel(:)'*ovel(:));
end
trace(iter).timingILQR = toc(t_ILQR);
clear X; xl = x0;
ndpc = floor(tc/dt);
for l = 1:ncts
    X(:,(l-1)*ndpc+1:l*ndpc+1)  = f_ode_show(xl,U(:,l),dt,tc,acon,vd,ec,eo,mo,Jo);
    xl = X(:,end);
end
ready = input('Are you ready?');
f_show_motion(X',[0:dt:tf],ndof,oposd,vd,eo);
gamma = X(15:18,1:end); disp(['integral of gamma = ',num2str(dt*sum(abs(gamma(:))))]);
trace(iter).totalGamma = dt*sum(abs(gamma(:)));
trace(iter).X = X; trace(iter).oposd = oposd;
if saveFlag
    save('trace_ilqr.mat','trace');
end
%% Functions
function [Xnew,Unew,cnew] = f_fwd_pass(x0,X,U,oposd,w,kff,Kfb,alpha)
% Forward pass
ilqr_params; global n m;
Xnew = zeros(n,ndts);  Unew = zeros(m,ncts); cnew = zeros(1,ndts);
Xnew(:,1) = x0;
ix = 1:n; iu = n+[1:m];
for l = 1:ncts
    dx = X(:,l) - Xnew(:,l);
    Unew(:,l) = U(:,l) + kff(:,l)*alpha - Kfb(:,:,l)*dx;
    Xnew(:,l+1) = f_ode_test(Xnew(:,l),Unew(:,l),dt,tc,acon,vd,ec,eo,mo,Jo);
    cnew(1,l) = f_ilqr_cost(Xnew(:,1:l),Unew(:,1:l),oposd,w,ndts);
end
cnew(1,ndts) = f_ilqr_cost(Xnew,Unew,oposd,w,ndts);
end

function [X,c,fx,fu,cx,cu,cxx,cxu,cuu] = f_diff_dyncst(x0,U,oposd,w,h)
% Numerical differentiation of dynamics and cost
ilqr_params; global n m;
X = x0;
ix = 1:n; iu = n+[1:m];
fJ = zeros(n,n+m,ncts); cJ = zeros(1,n+m,ndts); cJJ = zeros(n+m,n+m,ndts);
for l = 1:ndts
    xl = X(:,l); ul = U(:,l); Ul = U(:,1:l);
    if l < ndts
        fJ_dyn   = @(xu)f_ode_test(xu(ix),xu(iu),dt,tc,acon,vd,ec,eo,mo,Jo);
        fJ(:,:,l)  = f_num_diff(fJ_dyn,[xl;ul],h,cdiff);
    end
    fJ_cost  = @(xu) f_ilqr_cost(xu(ix,:),xu(iu,:),oposd,w,ndts);
    fJJ_cost = @(xu) squeeze(f_num_diff(fJ_cost,xu,h,cdiff));
    cJ(:,:,l)  = f_num_diff(fJ_cost,[X;Ul],h,cdiff);
    cJJ(:,:,l) = f_num_diff(fJJ_cost,[X;Ul],h,cdiff);
    c(1,l) = f_ilqr_cost(X,Ul,oposd,w,ndts);
    X(:,l+1) = f_ode_test(xl,ul,dt,tc,acon,vd,ec,eo,mo,Jo);
end
fx = fJ(:,ix,:); fu = fJ(:,iu,:);
cx = squeeze(cJ(:,ix,:));   cu = squeeze(cJ(:,iu,:));
cJJ = 0.5*(cJJ+permute(cJJ,[2 1 3]));
cxx = cJJ(ix,ix,:); cxu = cJJ(ix,iu,:); cuu = cJJ(iu,iu,:);
end

function [diverge,kff,Kfb,Vx,Vxx,dV] = f_bwd_pass(fx,fu,cx,cu,cxx,cxu,cuu,lambda,m,n,ncts,ndts,lb,ub,U)
% backward pass
kff = zeros(m,ncts);
Kfb = zeros(m,n,ncts);
Vx  = zeros(n,ndts);
Vxx = zeros(n,n,ndts);

Vx(:,ndts)  = cx(:,ndts);
Vxx(:,:,ndts) = cxx(:,:,ndts);

dV = [0 0];
diverge = 0;
for l = ncts:-1:1
    Qx  = cx(:,l) + fx(:,:,l)'*Vx(:,l+1);
    Qu  = cu(:,l) + fu(:,:,l)'*Vx(:,l+1);
    Qxx = cxx(:,:,l)  + fx(:,:,l)'*Vxx(:,:,l+1)*fx(:,:,l);
    Qux = cxu(:,:,l)' + fu(:,:,l)'*Vxx(:,:,l+1)*fx(:,:,l);
    Quu = cuu(:,:,l)  + fu(:,:,l)'*Vxx(:,:,l+1)*fu(:,:,l);
    
    Quu_f   = Quu + lambda*eye(m);
    Qux_reg = cxu(:,:,l)' + fu(:,:,l)'*Vxx(:,:,l+1)*fx(:,:,l);
    
    if isempty(lb) && isempty(ub)
        [R,d] = chol(Quu_f);
        if d, diverge = l; return; end
        kK = -R\(R'\[Qu Qux_reg]);
        kl = kK(:,1);
        Kl = kK(:,2:end);
    else
        Ulb = lb-U(:,l); Uub = ub-U(:,l);
        [kl,result,R,free] = boxQP(Quu_f,Qu,Ulb,Uub,kff);
        if result < 1, diverge = l; return; end
        
        Kl = zeros(m,n);
        if any(free)
            Lfree = -R\(R'\Qux_reg(free,:));
            Kl(free,:) = Lfree;
        end
    end
    
    dV         = dV  + [kl'*Qu, 0.5*kl'*Quu*kl];
    Vx(:,l)    = Qx  + Kl'*Quu*kl + Kl'*Qu  + Qux'*kl;
    Vxx(:,:,l) = Qxx + Kl'*Quu*Kl + Kl'*Qux + Qux'*Kl;
    Vxx(:,:,l) = 0.5*(Vxx(:,:,l) + Vxx(:,:,l)');
    
    kff(:,l)   = kl;
    Kfb(:,:,l) = Kl;
end
end
