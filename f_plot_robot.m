function f_plot_robot(q,ci,ndof,pbr,cc)
% Plot the robot given configuration q and color code index ci
global hlink hjnt hee hbase;

pl = f_link_pos(q);

pb = pbr;
plb = [pb';pl];
for j = 1:ndof
    hlink(j,ci) = plot([plb(j,1),plb(j+1,1)],[plb(j,2),plb(j+1,2)],'color','r','linewidth',5);
end
hjnt(ci) = plot(plb(1:end-1,1),plb(1:end-1,2),'k.','markersize',20);
hee = plot(plb(end,1),plb(end,2),'color',[.2 .2 .2],'marker','.','markersize',25);
hbase = patch([0.3,0.5,0.5,0.3,0.3],[-0.05 -0.05 0 0 -0.05],[0,0,0,0,0],'facecolor','k','facealpha',0.15,'edgecolor','none');