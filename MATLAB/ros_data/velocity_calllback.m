function velocity_calllback(~,msg)
    global tip_vel_t f5 target_pose
    tip_vel_t=[tip_vel_t,msg.Data];
    figure(f5);
    plot(tip_vel_t(1,:))
    hold on
    plot(tip_vel_t(2,:))
    plot(target_pose(3,:),"k")
    plot(target_pose(4,:),"c")
    legend('vx','vy','vxdes','vydes','Location','southwest')
    hold off
end