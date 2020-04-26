classdef UR3Model < handle % setup and move the UR3 robot, as well as log its transforms
    properties
        model;
        currentJoints;
        location;
        workspace;
        plyData;   
        name;
        pointCloud;
        qValueMatrix;
    end
    
    methods
        function self = UR3Model(name,workspace,location,draw)
            self.workspace = workspace;
            self.getRobot(name);
            self.currentJoints = zeros(1,6);
            self.model.base = location;
            self.location = location;
            self.name = name;
            
            if draw
                self.PlotAndColour(self.location);
            end
            
        end
        
        function [pointCloud] = GeneratePointCloud(self, stepSize)
            stepRads = deg2rad(stepSize);
            qlim = self.model.qlim;
            % Don't need to worry about joint 6
            pointCloudeSize = prod(floor((qlim(1:5,2)-qlim(1:5,1))/stepRads + 1));
            qValueMatrix = zeros(pointCloudeSize,6);
            pointCloud = ones(pointCloudeSize,3) .* self.model.base(1:3,4)';
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
%                               for q6 = qlim(6,1):stepRads:qlim(6,2)
                                    q = [q1,q2,q3,q4,q5,q6];
                                    if(self.withinBounds(q) == 1)
                                        qValueMatrix(counter,:) = q;
                                        tr = self.model.fkine(q);                        
                                        pointCloud(counter,:) = tr(1:3,4)';
                                        counter = counter + 1;                                    
                                    end
                                    count = count + 1;
                                    if mod(count/pointCloudeSize * 100,1) == 0
                                        display(['After ',num2str(toc),' seconds, completed ',num2str(count/pointCloudeSize * 100),'% of poses']);
                                    end
%                               end
                            end
                        end
                    end
                end
            end
%             ReducePointCloud(pointCloud);    

            for i = 1 : pointCloudeSize
                if pointCloud(i,:) == self.model.base(1:3,4)'
                    pointCloud = pointCloud(1:i-1,:);
                    break
                end
            end
            self.pointCloud = pointCloud;
            self.qValueMatrix = qValueMatrix;
            save('PcloudReduced','pointCloud','qValueMatrix');
        end
        
        function [pCloud] = LoadPointCloud(self)
            temp = load('PcloudReduced','pointCloud','qValueMatrix');
            self.pointCloud = temp.pointCloud;
            self.qValueMatrix = temp.qValueMatrix;
            pCloud = self.pointCloud;
        end
        
        function ReducePointCloud(self, pointCloud)
            [pCloudSize, c] = size(pointCloud);

            for i = 1 : pCloudSize
                if pointCloud(i,:) == self.model.base(1:3,4)'
                    pointCloud = pointCloud(1:i-1,:);
                    break
                end
            end
            self.pointCloud = pointCloud;
            save('PcloudReduced','pointCloud');
        end
        
        function Vol = MaxRobotVolume(self)
            if isempty(self.pointCloud)
                Vol = 0;
                display('no point cloud generated for this model yet');
            else
                [k, Vol] = convhull(self.pointCloud);
            end             
        end
        
        function [Reach, index] = MaxRobotReach(self)
            if isempty(self.pointCloud)
                Reach = 0;
                display('no point cloud generated for this model yet');
            else
                [r,c] = size(self.pointCloud);
                output = zeros(r,1);

                for i=1:r
                    output(i,1) = norm(self.model.base(1:3,4)' - self.pointCloud(i,:));
                end

                [Reach,index] = max(output);
            end             
        end           
        
        function PlotAndColour(self,location)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end
            
            % Display robot
            self.model.plot3d(self.currentJoints,'workspace',self.workspace,'floorlevel', 0);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight;
            end
            
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end    
        end
        
        function getRobot(self, name) % Setup Robot Parameters
            pause(0.001);
            L1 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]));
            L2 = Link('d',0,'a',-0.24365,'alpha',0,'qlim',deg2rad([-180 0]));
            L3 = Link('d',0,'a',-0.21325,'alpha',0,'qlim',deg2rad([-180 180]));
            L4 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]));
            L5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]));
            L6 = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360 360]));
            
            pause(0.0001)
            self.model = SerialLink([L1 L2 L3 L4 L5 L6], 'name', name);             
        end
        
        function [t] = withinBounds(self, q)
%             self.currentJoints = self.model.getpos();
            Joints = q;
            t = 1;
            [c,r] = size(Joints);
            
            %set current cords to be base location
            current_link = self.location;
            
            %iterate through all link pose to find if any go under the
            %table
            for joint = 1:r
                current_link = current_link * self.model.A(joint,Joints);
                if(current_link(3,4) < 0)
                    %if true return 0
                    t = 0;
                    return
                end
            end
        end
    end
end

% leftBaseP = [leftBase(1,4),leftBase(2,4),leftBase(3,4)]
% rightArmP = [rightArm(1,4),rightArm(2,4),rightArm(3,4)]
% 
% A = leftBaseP;
% B = rightArmP;
% 
% result = norm( A - B )
% result * 1000