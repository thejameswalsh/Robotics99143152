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
        stepRads = deg2rad(30);
        qlim = UR3_1.model.qlim;
        % Don't need to worry about joint 6
        pointCloudeSize = prod(floor((qlim(1:5,2)-qlim(1:5,1))/stepRads + 1));
        pointCloud = ones(pointCloudeSize,3) .* UR3_1.model.base(1:3,4)'
        counter = 1;
        count = 1;
        tic

        for q1 = qlim(1,1):stepRads:qlim(1,2)
            for q2 = qlim(2,1):stepRads:qlim(2,2)
                for q3 = qlim(3,1):stepRads:qlim(3,2)
                    for q4 = qlim(4,1):stepRads:qlim(4,2)
                        for q5 = qlim(5,1):stepRads:qlim(5,2)
                            % Don't need to worry about joint 6, just assume it=0
                            q6 = 0;
%                           for q6 = qlim(6,1):stepRads:qlim(6,2)
                                q = [q1,q2,q3,q4,q5,q6];
                                if(UR3_1.withinBounds(q) == 1)
                                    tr = UR3_1.model.fkine(q);                        
                                    pointCloud(counter,:) = tr(1:3,4)';
                                    counter = counter + 1;                                    
                                end
                                count = count + 1;
                                if mod(count/pointCloudeSize * 100,1) == 0
                                    display(['After ',num2str(toc),' seconds, completed ',num2str(count/pointCloudeSize * 100),'% of poses']);
                                end
%                        	end
                        end
                    end
                end
            end
        end
        save('PcloudReduced','pointCloud');
    case 1
        load('PcloudReduced');
end

% plot from tutorials 
% plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');

%% finding max reach

% [k, Max_Vol] = convhull(pointCloud);

% Max_Vol

hold off

close all

%% area through raidus and ideal angles
% 
% % find the top position to get the R
% UR3_1q = [0,90,0,90,90,0];
% endf = UR3_1.model.fkine(deg2rad(UR3_1q));
% maxlen_top = abs( endf(1:3,4)' - [0,0,0.152] );
% 
% % find the bottom position that will find h
% UR3_1q = [0,180,90,90,90,0];
% endf = UR3_1.model.fkine(deg2rad(UR3_1q));
% maxlen_bottom = endf(1:3,4)' - [0,0,0.152];
% 
% % max without taking anything out
% % would mean arm bent back on itself 
% Max_vol = ( 4 * pi * maxlen_top(3)^3 ) / 3
% 
% % doesn't yet include floor must ask
% % https://mathworld.wolfram.com/SphericalCap.html?fbclid=IwAR13igaa2iavUjBd3vyHoOVTVOrUEAFUITtciWRmLwOi0T3nV4ReMzjtfS8
% h = maxlen_top(3) - maxlen_bottom(3);
% Max_vol = (pi * h^2 * (3 * maxlen_top(3) - h)) / 3;
% 
% % 230 is only for the mdl arm replace with math from link to work out
% % variable for first arm
% redundant_vol = pi * 0.230^2 *  abs(maxlen_bottom(3));
% Max_vol = Max_vol - redundant_vol

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