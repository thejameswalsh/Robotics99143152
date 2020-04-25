%% size of cloud

plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');

%% finding max reach

'first method'
load('Pcloud');

[k, Max_Vol] = convhull(pointCloud);

Max_Vol

%%
'second method'
load('PcloudReduced');

[k, Max_Vol] = convhull(pointCloud);

Max_Vol
%%
'third method'
[pCloudSize, c] = size(pointCloud);

for i = 1 : pCloudSize
    if pointCloud(i,:) == [0,0,0]
        pointCloud = pointCloud(1:i-1,:);
        break
    end
end

[k, Max_Vol] = convhull(pointCloud);

Max_Vol

%% check out max reach

[r,c] = size(pointCloud);

a = [0.4000,0.2000,0];
b = pointCloud;

out = zeros(r,1);

for i=1:r
    out(i,1) = norm(a - b(i,:));
end

[M,I] = max(out)
out(514)

