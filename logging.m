function logging(Models)
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
    
    fclose(file);
end

end

