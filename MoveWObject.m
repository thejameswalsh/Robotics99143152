function [outputArg1,outputArg2] = MoveWObjects(Robot_Arm,Objects)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

size(Objects,2)
return

zoffset = -0.1;
% animate 1
goalQ = Robot_Arm.model.ikcon(Object(1).model.base * trotx(pi) * transl(0,0,zoffset),Robot_Arm.model.getpos);
jointTrajectory = jtraj(Robot_Arm.model.getpos(), goalQ,30);

for trajStep = 1:size(jointTrajectory,1)
    q = jointTrajectory(trajStep,:);
    Robot_Arm.model.animate(q);
    for i = 0:size(Objects,2)
        
    end
    pause(0.01);
end
pause(0.01);

goalQ = Robot_Arm.model.ikcon(Object(1).model.base * trotx(pi),Robot_Arm.model.getpos);
jointTrajectory = jtraj(Robot_Arm.model.getpos(), goalQ,30);

for trajStep = 1:size(jointTrajectory,1)
    q = jointTrajectory(trajStep,:);
    Robot_Arm.model.animate(q);
    pause(0.01);
end
pause(0.01);

end

