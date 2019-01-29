function X = f_ode_show(xk,uk,dt,tc,acon,vd,ec,eo,mo,Jo)

Xode = ode1(@(t,x) f_dyn_test(x,uk,dt,acon,vd,ec,eo,mo,Jo), [0:dt:tc], xk);
X = Xode';