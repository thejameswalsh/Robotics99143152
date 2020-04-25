%% assigntment one info dump

%% init
close all
set(0,'DefaultFigureWindowStyle','docked')
clear
clc

%% modelling UR3 Robot 1

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
        y = 0.5;
        z = 0.1;
end

% UR3_1 = SerialLink(linkList,'name','UR3_1','base',transl(x,y,z));
UR3_1 = UR3Model(workspace, transl(x,y,z), 1);

% UR3_1.model.teach()
% UR3_1.model.teach();

% UR3_1q = zeros(size(linkList));        
% UR3_1q = [0,90,0,90,0,0];
% endf = UR3_1.model.fkine(deg2rad(UR3_1q));

% UR3_1.model.plot3d(deg2rad(UR3_1q));
hold on

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
% UR3_2.plot3d(deg2rad(UR3_2q));
% hold on

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
        pointCloud = zeros(pointCloudeSize,3);
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

% 2.6 Create a 3D model showing where the end effector can be over all these samples.  
plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');

%% finding max reach

[k, Max_Vol] = convhull(pointCloud);

Max_Vol

%% finding max reach

% side view
UR3_1q = [0,90,0,90,0,0];
UR3_1.model.plot3d(deg2rad(UR3_1q));

% prompt = 'press enter to continue';
% input(prompt);

% top view
UR3_1q = [0,0,0,90,0,0];
UR3_1.model.plot3d(deg2rad(UR3_1q));
hold on

% prompt = 'press enter to continue';
% input(prompt);

%% area
% REDUNDANT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% UR3_1q = [0,180,0,90,90,0];
% endf = UR3_1.model.fkine(deg2rad(UR3_1q));
% 
% maxlen_right = endf(1:3,4)';
% 
% UR3_1q = [0,0,0,90,90,0];
% endf = UR3_1.model.fkine(deg2rad(UR3_1q));
% 
% maxlen_left = endf(1:3,4)';
% 
% result = norm( maxlen_left - maxlen_right ) / 2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% find the top position to get the R
UR3_1q = [0,90,0,90,90,0];
endf = UR3_1.model.fkine(deg2rad(UR3_1q));
maxlen_top = abs( endf(1:3,4)' - [0,0,0.152] );

% find the bottom position that will find h
UR3_1q = [0,180,90,90,90,0];
endf = UR3_1.model.fkine(deg2rad(UR3_1q));
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
UR3_1.model.plot3d(UR3_1q)

tempq = UR3_1.model.ikcon(housing_bottom_pose);

UR3_1.model.teach();
% animate 1 
jointTrajectory = jtraj(UR3_1.model.getpos(), tempq,100);

for trajStep = 1:size(jointTrajectory,1)
    q = jointTrajectory(trajStep,:);
    UR3_1.model.animate(q);
    pause(0.01);
    trajStep
end
pause(0.01);
% % back to start
% tempq = UR3_1.model.ikcon(housing_top_pose,[1,1,0,0,1,1])
% % animate 2
% jointTrajectory = jtraj(UR3_1.model.getpos(), zeros(size(linkList)),60)
% 
% for trajStep = 1:size(jointTrajectory,1)
%     q = jointTrajectory(trajStep,:);
%     UR3_1.model.animate(q);
%     pause(0.001);
%     trajStep
% end


tempq = UR3_1.model.ikcon(housing_top_pose,[1,1,1,0,0,0]);
% animate 2
jointTrajectory = jtraj(UR3_1.model.getpos(), tempq,60)

for trajStep = 1:size(jointTrajectory,1)
    q = jointTrajectory(trajStep,:);
    UR3_1.model.animate(q);
    pause(0.001);
    trajStep;
end

%% have a play
"done"

UR3_1.model.teach();