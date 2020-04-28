%% assigntment one info dump

%% init
close all
set(0,'DefaultFigureWindowStyle','docked')
clear
clc

%% modelling UR3 Robots
log = logger('log.txt');
hold on
floorOffset = -0.8905/2; %messured from bounding box
workSize = 1;
workspace = [-workSize workSize -workSize workSize (2*floorOffset) workSize];
scale = 0;

% enter base location information
XEditField.Value = -0.23;
YEditField.Value = 0.23;
ZEditField.Value = 0;

XEditField_2.Value = 0.23;
YEditField_2.Value = 0.23;
ZEditField_2.Value = 0;

XEditField_3.Value = -0.42;
YEditField_3.Value = 0.46;
ZEditField_3.Value = 0;

XEditField_4.Value = 0.42;
YEditField_4.Value = 0.46;
ZEditField_4.Value = 0;

XEditField_5.Value = 0.2;
YEditField_5.Value = 0.5;
ZEditField_5.Value = 0;

XEditField_6.Value = -0.05;
YEditField_6.Value = -0.1;
ZEditField_6.Value = 0;

% import arms
UR3_1 = UR3Model('UR3_1',workspace, transl(XEditField.Value,YEditField.Value,ZEditField.Value), 1);
UR3_2 = UR3Model('UR3_2',workspace, transl(XEditField_2.Value,YEditField_2.Value,ZEditField_2.Value), 1);

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
HousingTop = Objects('housingTop','1',workspace,transl(XEditField_3.Value,YEditField_3.Value,ZEditField_3.Value));
HousingBottom = Objects('housingBottom','1',workspace,transl(XEditField_4.Value,YEditField_4.Value,ZEditField_4.Value));
CircuitBoard = Objects('circuitBoard','1',workspace,transl(XEditField_5.Value,YEditField_5.Value,ZEditField_5.Value));
Crate = Objects('Crate','1',workspace,transl(XEditField_6.Value,YEditField_6.Value,ZEditField_6.Value));

pause(0.01);
% bodies = [UR3_1,UR3_1,Table,Fence1,Fence2,Fence2_2,Fence3,Fence4,Fence4_4,HousingTop,HousingBottom,CircuitBoard,Crate];
bodies = [UR3_1,UR3_2]
objects = [HousingTop,HousingBottom,CircuitBoard,Crate];
%log bodies
log.firstLogRobot(bodies);
log.firstLogRobot(objects);


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

%% log section

log.logging(bodies);
log.logging(objects);

%% place items down?

% temp = UR3_1;
% UR3_1 = UR3_2;
% UR3_2 = temp;
% delete(temp);

zoffset = -0.1;

handoffLocation = ((UR3_1.model.base*UR3_2.model.base)/2);
handoffLocation = handoffLocation(1:3,4);
if (max(handoffLocation)/2) < 0.3
    handoffLocation = transl(handoffLocation) * transl(0,0,0.3);
else
    handoffLocation = transl(handoffLocation) * transl(0,0,max(handoffLocation)/2);
end

handoffLocation = handoffLocation * trotx(pi/2) * troty(0);

hold on

% animate 1, pick up top Housing
% going to housing top carrying nothing with arm one
MoveWObjects(UR3_1,HousingTop.model.base, []);
log.logging(bodies);
log.logging(objects);

% animate 2, pick up bottom Housing
% going to housing bottom carrying nothing with arm two
MoveWObjects(UR3_2,CircuitBoard.model.base, []);
log.logging(bodies);
log.logging(objects);

% animate 3, go to meeting location to put parts together
% carrying top housing with arm one
%             MoveWObjects(UR3_1,[0,-pi/4,pi/2,0,0,0], [HousingTop]);
MoveWObjects(UR3_1,handoffLocation, [HousingTop]);
log.logging(bodies);
log.logging(objects);

% animate 4, go to UR3_1 pose location to put parts together
% carrying bottom housing with arm two
MoveWObjects(UR3_2,[0,-(3*pi)/4,pi/2,0,0,0], [CircuitBoard])
MoveWObjects(UR3_2,UR3_1.model.fkine(UR3_1.model.getpos)* trotz(pi), [CircuitBoard]);
log.logging(bodies);
log.logging(objects);

% animate 5, go to circuit pose location to pickup
% carrying nothing with arm two
% go back to init position
%             MoveWObjects(UR3_2,[-pi,-pi/2,pi/2,0,0,0], []);
MoveWObjects(UR3_2,HousingBottom.model.base, []);
log.logging(bodies);
log.logging(objects);

% animate 6, go to UR3_1 pose location to put parts together
% carrying bottom housing with arm two
MoveWObjects(UR3_2,UR3_1.model.fkine(UR3_1.model.getpos)* trotz(pi), [HousingBottom]);
log.logging(bodies);
log.logging(objects);

% animate 6, go to crate pose to dump parts
% carrying bottom housing top housing and circuit board
Qpass = UR3_1.model.getpos;
Qpass(2) = -pi/2;
MoveWObjects(UR3_1,[-pi,-pi/2,pi/2,0,0,0], [HousingTop, HousingBottom, CircuitBoard]);
MoveWObjects(UR3_1,Crate.model.base * transl(0,0,0.1), [HousingTop, HousingBottom, CircuitBoard]);
log.logging(bodies);
log.logging(objects);

%template
% MoveWObjects(UR3_1,transl(0,0,0), [HousingTop, HousingBottom])


%% have a play
"done"

UR3_1.model.teach();

