function f_show_motion(x,tspan,ndof,oposd,vd,eo)
% Show motions of the robot and the object given q_ode
cc = [0 0 1; 1 0 0];        % color codes for robot plotting
pbl = [-0.4; 0];            % [m,m] position of the base of left arm
pbr = [0.4;  0];            % [m,m] position of the base of right arm
%% Figure settings
figure(1); clf;
set(gcf,'Position',[100 50 1285 725],'PaperUnits','points','Renderer','painters','color','white');
hold on; grid on; box on; axis equal;
xlim([-.2 1]); ylim([-.05 1]);
set(gca,'XTick',-1:0.2:1,'YTick',-1:0.2:1,'FontSize',20);%,'FontName','cmr10');
set(gca,'TickLabelInterpreter','latex');
xlabel('x [m]','interpreter','latex','FontSize',20);
ylabel('y [m]','interpreter','latex','FontSize',20);
%% Plot settings
global hlink hjnt hee hbase;
href = plot(oposd(1),oposd(2),'ro','markersize',12);
hobj0 = plot(x(1,9),x(1,10),'bx','markersize',12);
legend([href,hobj0],{'Desired position','Actual position'},...
                     'Interpreter','latex','AutoUpdate','off','Location','ne');
legend boxoff;
delete(hobj0);
dt = tspan(2)-tspan(1);
%% Let the show begin
for i = 1:length(tspan)
    qpos = x(i,1:4); opos = x(i,9:10)'; orot = x(i,11);
    vp = opos*ones(1,4) + f_rot2d(orot)*vd*eo/2;
    if i>1, delete(hlink(:,1)); delete(hjnt); delete(hee); delete(hbase); delete(hobj); delete(hbox); end
    f_plot_robot(qpos,1,ndof,pbr,cc);
    hobj = plot(opos(1),opos(2),'bx','markersize',12);
    hbox = patch([vp(1,1:4),vp(1,1)],[vp(2,1:4),vp(2,1)],[0,0,0,0,0],'facecolor','b','facealpha',0.15,'edgecolor','b');
    title(['t = ',num2str(round(tspan(i)*1e2)/1e2),' s'],'interpreter','latex');
    pause(dt);
    if i == 1
        ready = input('Are you ready?');
    end
end