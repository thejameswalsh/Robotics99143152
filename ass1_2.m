%% assigntment one info dump

%% init
close all
set(0,'DefaultFigureWindowStyle','docked')
clear
clc

%% modelling UR3 Robots

% hold on
% m = PlaceObject('fence.ply',[1,1,0])
% hold on
% delete(m)
% m = PlaceObject('fence.ply',[2,1,10])
% n = PlaceObject('fenceRotate.ply',[2,1,10])
% pause(0.1);
hold on
floorOffset = -0.8905/2; %messured from bounding box
workSize = 3;
workspace = [-workSize workSize -workSize workSize (2*floorOffset) workSize];
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

% import arms
UR3_1 = UR3Model('UR3_1',workspace, transl(x,y,z), 1);
UR3_2 = UR3Model('UR3_2',workspace, transl(-0.1,-0.05,z), 1);

out = ( UR3_1.model.base(1:2,4) + UR3_2.model.base(1:2,4) ) .'/2;
safeDistance = 1.5;
fenceOffset = 1.35
tableOffset = 0.5
% import objects
% fix translations
Table = Objects('table','1',workspace,transl(out(1), out(2),floorOffset));
Fence1 = Objects('fence','1',workspace,transl(out(1),out(2) + safeDistance,floorOffset));
Fence2 = Objects('fence','2',workspace,transl(out(1) - tableOffset + safeDistance,out(2) + fenceOffset/2,floorOffset) * trotz(pi/2));
Fence2_2 = Objects('fence','2_2',workspace,transl(out(1) - tableOffset + safeDistance,out(2) - fenceOffset/2,floorOffset) * trotz(pi/2));
Fence3 = Objects('fence','3',workspace,transl(out(1),out(2) - safeDistance,floorOffset) * trotz(pi));
Fence4 = Objects('fence','4',workspace,transl(out(1) + tableOffset - safeDistance,out(2) + fenceOffset/2,floorOffset) * trotz((3 *pi)/2));
Fence4_4 = Objects('fence','4_4',workspace,transl(out(1) + tableOffset - safeDistance,out(2) - fenceOffset/2,floorOffset) * trotz((3 *pi)/2));
HousingTop = Objects('housingTop','1',workspace,transl(-0.2,0.2,z));
HousingBottom = Objects('housingBottom','1',workspace,transl(0.5,0.3,z));
CircuitBoard = Objects('circuitBoard','1',workspace,transl(0,-0.2,z));
Create = Objects('Crate','1',workspace,transl(0,0.4,z));

pause(0.01);

%% Modelling bodies 

%% finding max reach Point cloud

% prompt = 'Select a robot model; MDL = 1 PERSONAL = 2: ';
% chunk = input(prompt);
chunk = 0;
switch chunk
% 2.4 Sample the joint angles within the joint limits at 30 degree increments between each of the joint limits
% & 2.5 Use fkine to determine the point in space for each of these poses, so that you end up with a big list of points
    case 0
        pointCloud = UR3_1.GeneratePointCloud(45);
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
        UR3_1.model.animate(UR3_1.qValueMatrix(Index,:));
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
        pointCloudPlot = plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
        prompt = 'Press enter when done with demo';
        input(prompt);
        delete(pointCloudPlot)
end

%% finish section

% hold off

% close all

%% place items down?

temp = UR3_1;
UR3_1 = UR3_2;
UR3_2 = UR3_1;
delete(temp);

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

% init pos
initQ = [0,0,0,0,0,0]; 

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

