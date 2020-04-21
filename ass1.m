%% assigntment one info dump

%% init
close all
set(0,'DefaultFigureWindowStyle','docked')
clear
clc

%% modelling UR3 Robot 1

% prompt = 'Select a robot model; MDL = 1 PERSONAL = 2: ';
% chunk = input(prompt);
chunk = 1;
switch chunk
    case 1 %MDL
        L1 = Link('d',0.152,'a',0.0,'alpha',deg2rad(90),'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
        L2 = Link('d',0.0,'a',0.230,'alpha',deg2rad(0),'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
        L3 = Link('d',0.0,'a',0.213,'alpha',deg2rad(0),'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
        L4 = Link('d',0.0,'a',0.0,'alpha',deg2rad(90),'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
        L5 = Link('d',0.083,'a',0.0,'alpha',deg2rad(-90),'offset',pi/2,'qlim',[deg2rad(-360),deg2rad(360)]);
        L6 = Link('d',0.082,'a',0.0,'alpha',deg2rad(0),'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
    case 2 %PERSONAL
        L1 = Link('d',0.152,'a',0,'alpha',deg2rad(90),'offset',0);
        L2 = Link('d',-0.120,'a',0.230,'alpha',deg2rad(0),'offset',0);
        L3 = Link('d',0.093,'a',0.213,'alpha',deg2rad(-180),'offset',0);
        L4 = Link('d',0.083,'a',0.0,'alpha',deg2rad(90),'offset',0);
        L5 = Link('d',0.083,'a',0.0,'alpha',deg2rad(90),'offset',0);
        L6 = Link('d',0.082,'a',0.0,'alpha',deg2rad(0),'offset',0);
end

linkList = [L1 L2 L3 L4 L5 L6];
size(linkList);

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
        x = 0;
        y = 0;
        z = 0;
end

UR3_1 = SerialLink(linkList,'name','UR3_1','base',transl(x,y,z));
UR3_1.teach();

UR3_1q = zeros(size(linkList));        
UR3_1q = [0,90,0,90,0,0];
endf = UR3_1.fkine(deg2rad(UR3_1q));

UR3_1.plot(deg2rad(UR3_1q));
hold on

% a = UR3_1.fkine(UR3_1q)
% 0.540 + 0.152 - a(3,4)
% 0.540 + 0.152

%% modelling UR3 Robot 2

% % enter base location information
% init = 1;
% switch init
%     case 0 
%         prompt = 'Select an inital x: ';
%         x = input(prompt);
%         prompt = 'Select an inital y: ';
%         y = input(prompt);
%         prompt = 'Select an inital z: ';
%         z = input(prompt);
%     case 1
%         x = 0.5;
%         y = 0.5;
%         z = 0;
% end
% 
% UR3_2 = SerialLink(linkList,'name','UR3_2','base',transl(x,y,z));
%     
% UR3_2q = [0,90,0,90,0,0];
% endf = UR3_2.fkine(deg2rad(UR3_2q));
% 
% UR3_2.plot(deg2rad(UR3_2q));
% hold on

%% finding max reach

% side view
UR3_1q = [0,90,0,90,0,0];
UR3_1.plot(deg2rad(UR3_1q));

% prompt = 'press enter to continue';
% input(prompt);

% top view
UR3_1q = [0,0,0,90,0,0];
UR3_1.plot(deg2rad(UR3_1q));
hold on

% prompt = 'press enter to continue';
% input(prompt);

%% area
% REDUNDANT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% UR3_1q = [0,180,0,90,90,0];
% endf = UR3_1.fkine(deg2rad(UR3_1q));
% 
% maxlen_right = endf(1:3,4)';
% 
% UR3_1q = [0,0,0,90,90,0];
% endf = UR3_1.fkine(deg2rad(UR3_1q));
% 
% maxlen_left = endf(1:3,4)';
% 
% result = norm( maxlen_left - maxlen_right ) / 2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% find the top position to get the R
UR3_1q = [0,90,0,90,90,0];
endf = UR3_1.fkine(deg2rad(UR3_1q));
maxlen_top = abs( endf(1:3,4)' - [0,0,0.152] );

% find the bottom position that will find h
UR3_1q = [0,180,90,90,90,0];
endf = UR3_1.fkine(deg2rad(UR3_1q));
maxlen_bottom = endf(1:3,4)' - [0,0,0.152];

% max without taking anything out
% would mean arm bent back on itself 
Max_vol = ( 4 * pi * maxlen_top(3)^3 ) / 3

% doesn't yet include floor must ask
% https://mathworld.wolfram.com/SphericalCap.html?fbclid=IwAR13igaa2iavUjBd3vyHoOVTVOrUEAFUITtciWRmLwOi0T3nV4ReMzjtfS8
h = maxlen_top(3) - maxlen_bottom(3);
Max_vol = (pi * h^2 * (3 * maxlen_top(3) - h)) / 3;

% 230 is only for the mdl arm replace with math from link to work out
% variable for first arm
redundant_vol = pi * 0.230^2 *  abs(maxlen_bottom(3));
Max_vol = Max_vol - redundant_vol
%% place items down?

housing_top_pose = transl(0,0.25,0) * trotx(pi)
housing_bottom_pose = transl(0.15,0.3,0) * trotx(pi)
circuit_board_pose = transl(-0.2,0.35,0) * trotx(pi)

trplot(housing_top_pose);
% hold on
trplot(housing_bottom_pose);
% hold on
trplot(circuit_board_pose);

% get pose
% init pos
UR3_1q = zeros(size(linkList)); 
UR3_1.plot(UR3_1q)

tempq = UR3_1.ikcon(housing_bottom_pose);

UR3_1.teach();
% animate 1 
jointTrajectory = jtraj(UR3_1.getpos(), tempq,100);

for trajStep = 1:size(jointTrajectory,1)
    q = jointTrajectory(trajStep,:);
    UR3_1.animate(q);
    pause(0.01);
    trajStep
end
pause(0.01);
% % back to start
% tempq = UR3_1.ikcon(housing_top_pose,[1,1,0,0,1,1])
% % animate 2
% jointTrajectory = jtraj(UR3_1.getpos(), zeros(size(linkList)),60)
% 
% for trajStep = 1:size(jointTrajectory,1)
%     q = jointTrajectory(trajStep,:);
%     UR3_1.animate(q);
%     pause(0.001);
%     trajStep
% end


tempq = UR3_1.ikcon(housing_top_pose,[1,1,1,0,0,0]);
% animate 2
jointTrajectory = jtraj(UR3_1.getpos(), tempq,60)

for trajStep = 1:size(jointTrajectory,1)
    q = jointTrajectory(trajStep,:);
    UR3_1.animate(q);
    pause(0.001);
    trajStep;
end

%% have a play
"done"

UR3_1.teach();