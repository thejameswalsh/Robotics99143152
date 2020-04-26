classdef Objects < handle % class to handle setting up of the static body
    properties
        model;
        faceData;
        vertexData;
        plyData;
        workspace;
        location;
    end
    
    methods
        function self = Objects(Modelname, workspace, location)
            self.plotAndColour(Modelname, workspace, location);
            camlight;
        end
               
        function plotAndColour(self, Modelname, workspace, location)
            if isempty(self.faceData) || isempty(self.vertexData) || isempty(self.plyData)
                [self.faceData,self.vertexData,self.plyData] = plyread([Modelname, '.ply'],'tri');
                % ply file data of bodys is stored for each body
            end
            
            L1 = Link('alpha',0,'a',1,'d',0,'offset',0);
            self.model = SerialLink(L1, 'name' , Modelname);
            % 1 link robot used to simulate bodys for simplicity
            self.model.faces = {self.faceData,[]};
            self.model.points = {self.vertexData,[]};
            
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

