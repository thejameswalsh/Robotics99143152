classdef logger
    %UNTITLED6 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        fileName;
    end
    
    methods
        function obj = logger(fileName)
            %UNTITLED6 Construct an instance of this class
            %   Detailed explanation goes here
            obj.fileName = fileName;
            file = fopen(fileName,'w');
            fprintf(file,'Welcome to 99143152 Robotics assignment 1\n');
            fprintf(file,'\n');
            fclose(file);
        end
        
        function firstLogRobot(obj, Models)
            for i = 1:size(Models,2)
                file = fopen('log.txt','a');
                fprintf(file,['Model Name: ',Models(i).model.name,'\n']);
                fprintf(file,'Model qlims: %.3f %.3f \n',Models(i).model.qlim);
                fprintf(file,'Model base x y z: %.4f %.4f %.4f \n', Models(i).model.base(1:3,4));
                fprintf(file,'\n');
                
                fclose(file);
            end
        end
        
        function MaxLog(obj, Models)
            for i = 1:size(Models,2)
                file = fopen('log.txt','a');
                fprintf(file,['Model Name: ',Models(i).model.name,'\n']);
                fprintf(file,'Max Reach : %.4f\n',Models(i).Max_Reach);
                fprintf(file,'Max Volume: %.4f\n', Models(i).Max_Vol);
                fprintf(file,'\n');
                
                fclose(file);
            end
        end
        
        function logging(obj, Models)
            %UNTITLED5 Summary of this function goes here
            %   Detailed explanation goes here
            for i = 1:size(Models,2)
                file = fopen('log.txt','a');
                pos = Models(i).model.fkine(Models(i).model.getpos);
                pos = pos(1:3,4);
                q = Models(i).model.getpos;
                fprintf(file,['Model Name: ',Models(i).model.name,'\n']);
                fprintf(file,'Model x y z: %.4f %.4f %.4f \n', pos);
                fprintf(file,'Model joints: %.3f %.3f %.3f %.3f %.3f %.3f \n',q);
                fprintf(file,'\n');
                
                fclose(file);
            end
            
        end
    end
end

