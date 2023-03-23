function tip_pose_calllback(~,msg)
    global tip_poses_t f1 target_pose
    tip_poses_t=[tip_poses_t,msg.Data];
    figure(f1);
    hold on
    plot(tip_poses_t(1,:))
    plot(tip_poses_t(2,:))
    plot(target_pose(1,:),"k")
    plot(target_pose(2,:),"c")
    hold off
    legend('x','y','xdes','ydes','Location','southwest')


end