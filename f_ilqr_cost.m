function c = f_ilqr_cost(X,U,oposd,w,ndts)
% returns cost value given time and states
%% Parse
l = size(X,2);
qvel = X(5:8,:); ovel = X(12:14,:); gamma = X(15:18,:); kcon  = U(5:8,:);
%% Costs
% integrated cost
cvel   = qvel(:)'*qvel(:)+ovel(:)'*ovel(:);
% cvel   = qvel(:)'*qvel(:);
cgamma = gamma(:)'*gamma(:);
if isnan(kcon(1,end))
    kcon(:,end) = [];
    ckcon = kcon(:)'*kcon(:);
else
    ckcon  = kcon(:)'*kcon(:);
end
%  terminal cost
cposf = 0;
if l == ndts
    oposf  = X(9:10,end);
    cposf = (oposd-oposf)'*(oposd-oposf);
end
    
c = w(1)*cposf + w(2)*cvel + w(3)*cgamma + w(4)*ckcon;