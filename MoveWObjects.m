function [RobotEndPose,Objects] = MoveWObjects(Robot_Arm,GoalPose,Objects)
%MoveWObjects Summary of this function goes here
%   Takes in a robot arm, a pose and a matrix of objects and plots them
%   Returning the robot endeffector pose and the objects poses in a same
%   size matrix
% ObjectsPoses = zeros;
zoffset = -0.1;
% animate 1
initQ = Robot_Arm.model.getpos .* [1,0,0,0,0,0];
initQ(2) = pi/2;
if size(GoalPose,2) == 6
    goalQ = GoalPose;
else
%     goalQ = Robot_Arm.model.ikcon(GoalPose * trotx(pi) * transl(0,0,zoffset),initQ);
    goalQ = Robot_Arm.model.ikcon(GoalPose * trotx(pi) * transl(0,0,zoffset),Robot_Arm.model.getpos);
end
jointTrajectory = jtraj(Robot_Arm.model.getpos(), goalQ,30);

for trajStep = 1:size(jointTrajectory,1)
    q = jointTrajectory(trajStep,:);
    Robot_Arm.model.animate(q);
    if isempty(Objects) == 0
        newBase = Robot_Arm.model.fkine(q);
    end
    for i = 1:size(Objects,2)
        Objects(i).model.base = newBase;
        Objects(i).model.animate(0);
    end
    pause(0.01);
end
pause(0.01);

% initQ = Robot_Arm.model.getpos .* [1,0,0,0,0,0];
% initQ(2) = 90;
if size(GoalPose,2) == 6
    return
else
    goalQ = Robot_Arm.model.ikcon(GoalPose * trotx(pi),Robot_Arm.model.getpos);
end
jointTrajectory = jtraj(Robot_Arm.model.getpos(), goalQ,30);

for trajStep = 1:size(jointTrajectory,1)
    q = jointTrajectory(trajStep,:);
    Robot_Arm.model.animate(q);
    if isempty(Objects) == 0
        newBase = Robot_Arm.model.fkine(q);
    end
    for i = 1:size(Objects,2)
        Objects(i).model.base = newBase;
        Objects(i).model.animate(0);
    end
    pause(0.01);
end
pause(0.01);

RobotEndPose = Robot_Arm.model.fkine(Robot_Arm.model.getpos);
end

