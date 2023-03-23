function slope_calllback(~,msg)
    global slopes_t f4 slopes_des TimeStepCount
    slopes_t=[slopes_t,msg.Data];
    figure(f4);
    plot(slopes_t(1,:))
    hold on
    plot(slopes_t(2,:))
    plot(slopes_t(3,:))
    plot(slopes_t(4,:))
    plot(slopes_t(5,:))
%     plot(slopes_des(1:TimeStepCount))
%     plot(tip_poses_t(3,:))
%     legend('x','y','slope')
    legend('q1','q2','q3','q4','q5')
    hold off
end