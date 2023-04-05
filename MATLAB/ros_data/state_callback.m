function state_callback(~,msg)
    global xt yt nColsNodes original_pose f2 t_x t_y r e tip_vel_t target_pose
    x=msg.Data(1:2:end);
    y=msg.Data(2:2:end);
    xt=[xt,x];
    yt=[yt,y];
    x=x+original_pose(:,1);
    y=y+original_pose(:,2);
    figure(f2);
    plot(x(1:nColsNodes:end),y(1:nColsNodes:end),'k','LineWidth',2) %line1
    hold on
    plot(t_x, -0.75+ t_y, 'x', 'LineWidth', 4, 'MarkerSize', 10);
    plot(x(nColsNodes:nColsNodes:end),y(nColsNodes:nColsNodes:end),'k','LineWidth',2) %line3 flip
    plot(x(ceil(nColsNodes*0.5):nColsNodes:end),y(ceil(nColsNodes*0.5):nColsNodes:end),'k--','LineWidth',1)
    plot(x((end-nColsNodes+1):end),y((end-nColsNodes+1):end),'k','LineWidth',2) %line2
    plot(x(1:nColsNodes),y(1:nColsNodes),'b','LineWidth',4) %line4 flip
    theta=linspace(-3.14,3.14,200);
    % Generate x-coordinates.
    xr=r*sin(theta);

    % Generate y-coordinate.
    yr=r*sqrt(1-e^2)*(1-cos(theta));

    % plot the circle.
    plot(xr,yr - 0.75);
    hold off

    text(-0.6,0.4,sprintf('Radius=%.2f and Eccentricity=%.2f',r,e));
    tip_vel = sqrt((tip_vel_t(1,end))^2 + (tip_vel_t(2,end))^2);
    tar_vel = sqrt((target_pose(3,end))^2 + (target_pose(4,end))^2);
    text(-0.6,0.2,sprintf('Tip Velocity=%.2f and Target Velocity=%.2f',tip_vel, tar_vel));
    
    xlim([-0.05+original_pose(end,2),0.05-original_pose(end,2)])
    ylim([original_pose(end,2)*1.5,0.6])
end