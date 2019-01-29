function xf = f_ode_test(xk,uk,dt,tc,acon,vd,ec,eo,mo,Jo)

Xode = ode1(@(t,x) f_dyn_test(x,uk,dt,acon,vd,ec,eo,mo,Jo), [0:dt:tc], xk);
xf = Xode(end,:)';