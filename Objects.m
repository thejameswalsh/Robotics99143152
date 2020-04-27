classdef Objects < handle % class to handle setting up of the static body
    properties
        model;
        plyData;
        workspace;
        location;
    end
    
    methods
        function self = Objects(ModelName, ModelNum, workspace, location)
            self.plotAndColour(workspace, ModelName, ModelNum, location);
        end
               
        function plotAndColour(self, workspace, ModelName, ModelNum, location)
            for linkIndex = 0:1
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread([ModelName,'.ply'],'tri');
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end
            
            L1 = Link('alpha',0,'a',1,'d',0,'offset',0);
            self.model = SerialLink(L1, 'name' , [ModelName,'_', num2str(ModelNum)]);
            % 1 link robot used to simulate bodys for simplicity
            self.model.faces = {faceData,[]};
            self.model.points = {vertexData,[]};
            
            % Display body
            self.model.base = location;
            self.model.plot3d(0, 'workspace', workspace);
            
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
    end
end