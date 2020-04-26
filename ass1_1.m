%% assigntment one info dump

%% init
close all
set(0,'DefaultFigureWindowStyle','docked')
clear
clc

%% modelling UR3 Robots

hold on
workSize = 0.6;
workspace = [-workSize workSize -workSize workSize -workSize workSize];
scale = 0;

% enter base location information
init = 1;
switch init
    case 0 
        prompt = 'Select an inital x: ';
        x = input(prompt);
        prompt = 'Select an inital y: ';
        y = input(prompt);
        prompt = 'Select an inital z: ';
        z = input(prompt);
    case 1
        x = 0.4;
        y = 0.2;
        z = 0;
end

% UR3_1 = SerialLink(linkList,'name','UR3_1','base',transl(x,y,z));
UR3_1 = UR3Model('UR3_1',workspace, transl(x,y,z), 1);

UR3_2 = UR3Model('UR3_2',workspace, transl(-0.1,-0.05,z), 1);

pause(0.01);

%% Modelling bodies 

%% finding max reach Point cloud

% prompt = 'Select a robot model; MDL = 1 PERSONAL = 2: ';
% chunk = input(prompt);
chunk = 1;
switch chunk
% 2.4 Sample the joint angles within the joint limits at 30 degree increments between each of the joint limits
% & 2.5 Use fkine to determine the point in space for each of these poses, so that you end up with a big list of points
    case 0
        pointCloud = UR3_1.GeneratePointCloud(30);
    case 1
        pointCloud = UR3_1.LoadPointCloud();
end

%% Finding max reach

[Reach,Index] = UR3_1.MaxRobotReach();

Max_Reach = Reach

prompt = 'Would you like to plot max reach; 0 = NO 1 = YES ';
chunk = input(prompt);
switch chunk
% 2.4 Sample the joint angles within the joint limits at 30 degree increments between each of the joint limits
% & 2.5 Use fkine to determine the point in space for each of these poses, so that you end up with a big list of points
    case 0
        
    case 1
        UR3_1.model.plot3d(UR3_1.qValueMatrix(Index,:));
end



%% finding max reach

Max_Volume = UR3_1.MaxRobotVolume()

%% plot point cloud
 
prompt = 'Would you like to plot the point cloud; 0 = NO 1 = YES ';
chunk = input(prompt);
switch chunk
% 2.4 Sample the joint angles within the joint limits at 30 degree increments between each of the joint limits
% & 2.5 Use fkine to determine the point in space for each of these poses, so that you end up with a big list of points
    case 0
        
    case 1
        % plot from tutorials
        plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
end


%% finish section

hold off

close all

%% place items down?

zoffset = -0.1;

housing_top_location = [0.2,0,0]
housing_bottom_location = [0.3,0,0]
circuit_board_location = [-0.1,0.2,0]

handoffLocation = ((UR3_1.model.base*UR3_2.model.base)/2);
handoffLocation = handoffLocation(1:3,4);
if (max(handoffLocation)/2) < 0.25
    handoffLocation = transl(handoffLocation) * transl(0,0,0.25);
else
    handoffLocation = transl(handoffLocation) * transl(0,0,max(handoffLocation)/2);
end

housing_top_pose = transl(housing_top_location) * trotx(pi)
housing_bottom_pose = transl(housing_bottom_location) * trotx(pi)
circuit_board_pose = transl(circuit_board_location) * trotx(pi)

hold on

trplot(housing_top_pose,'length',0.1);
% hold on
trplot(housing_bottom_pose,'length',0.1);
% hold on
trplot(circuit_board_pose,'length',0.1);

% get pose
% init pos
initQ = [0,0,0,0,0,0]; 
UR3_1.model.plot3d(initQ)
UR3_2.model.plot3d(initQ)

% animate 1 
goalQ = UR3_1.model.ikcon(housing_top_pose * transl(0,0,zoffset),UR3_1.model.getpos);
jointTrajectory = jtraj(UR3_1.model.getpos(), goalQ,30);

for trajStep = 1:size(jointTrajectory,1)
    q = jointTrajectory(trajStep,:);
    UR3_1.model.animate(q);
    pause(0.01);
end
pause(0.01);

goalQ = UR3_1.model.ikcon(housing_top_pose,UR3_1.model.getpos);
jointTrajectory = jtraj(UR3_1.model.getpos(), goalQ,30);

for trajStep = 1:size(jointTrajectory,1)
    q = jointTrajectory(trajStep,:);
    UR3_1.model.animate(q);
    pause(0.01);
end
pause(0.01);

% animate 2
goalQ = UR3_2.model.ikcon(circuit_board_pose * transl(0,0,zoffset),UR3_2.model.getpos);
jointTrajectory = jtraj(UR3_2.model.getpos(), goalQ,30);

for trajStep = 1:size(jointTrajectory,1)
    q = jointTrajectory(trajStep,:);
    UR3_2.model.animate(q);
    pause(0.001);
    trajStep;
end

goalQ = UR3_2.model.ikcon(circuit_board_pose,UR3_2.model.getpos);
jointTrajectory = jtraj(UR3_2.model.getpos(), goalQ,30);

for trajStep = 1:size(jointTrajectory,1)
    q = jointTrajectory(trajStep,:);
    UR3_2.model.animate(q);
    pause(0.001);
    trajStep;
end

%% put together

% animate 1 
goalQ = UR3_1.model.ikcon(handoffLocation,UR3_1.model.getpos);
jointTrajectory = jtraj(UR3_1.model.getpos(), goalQ,30);

for trajStep = 1:size(jointTrajectory,1)
    q = jointTrajectory(trajStep,:);
    UR3_1.model.animate(q);
    pause(0.01);
end
pause(0.01);

% animate 2
goalQ = UR3_2.model.ikcon(handoffLocation * trotx(pi),UR3_2.model.getpos);
jointTrajectory = jtraj(UR3_2.model.getpos(), goalQ,30);

for trajStep = 1:size(jointTrajectory,1)
    q = jointTrajectory(trajStep,:);
    UR3_2.model.animate(q);
    pause(0.001);
    trajStep;
end

%% get third part

% animate 2
goalQ = UR3_2.model.ikcon(housing_bottom_pose * transl(0,0,zoffset),UR3_2.model.getpos);
jointTrajectory = jtraj(UR3_2.model.getpos(), goalQ,30);

for trajStep = 1:size(jointTrajectory,1)
    q = jointTrajectory(trajStep,:);
    UR3_2.model.animate(q);
    pause(0.001);
    trajStep;
end

goalQ = UR3_2.model.ikcon(housing_bottom_pose,UR3_2.model.getpos);
jointTrajectory = jtraj(UR3_2.model.getpos(), goalQ,30);

for trajStep = 1:size(jointTrajectory,1)
    q = jointTrajectory(trajStep,:);
    UR3_2.model.animate(q);
    pause(0.001);
    trajStep;
end

%% place together
% animate 2
goalQ = UR3_2.model.ikcon(handoffLocation * trotx(pi),UR3_2.model.getpos);
jointTrajectory = jtraj(UR3_2.model.getpos(), goalQ,30);

for trajStep = 1:size(jointTrajectory,1)
    q = jointTrajectory(trajStep,:);
    UR3_2.model.animate(q);
    pause(0.001);
    trajStep;
end

%% have a play
"done"

UR3_1.model.teach();

