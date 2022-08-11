clear; close all; clc

load('output.mat')

time = 0:0.01:(model.Tf-0.02);
numTrace = model.numTraces;
xState = Traces(:,2:end,2);
xStateBounds = reachTube(:,:,2);

yState = Traces(:,2:end,3);
yStateBounds = reachTube(:,:,3);

zState = Traces(:,2:end,4);
zStateBounds = reachTube(:,:,4);

h = zeros(1,3);
figure(1)
plot(time, xState, 'LineWidth',0.5,'Color','blue');
hold on
h(2:end) = plot(time, xStateBounds, 'LineWidth',1.5, 'Color','black');
hold off
ylabel('x in meters')
xlabel('time in seconds')
legend(h(2:end),'Upper Bound','Lower Bound', 'Location','northwest')
title('Reachable set of the aircraft (state x)') 

h = zeros(1,3);
figure(2)
plot(time, yState, 'LineWidth',0.5,'Color','blue');
hold on
h(2:end) = plot(time, yStateBounds, 'LineWidth',1.5, 'Color','black');
hold off
ylabel('y in meters')
xlabel('time in seconds')
legend(h(2:end),'Upper Bound','Lower Bound', 'Location','northwest')
title('Reachable set of the aircraft (state y)') 

h = zeros(1,3);
figure(3) 
plot(time, zState, 'LineWidth',0.5,'Color','blue');
hold on
h(2:end) = plot(time, zStateBounds, 'LineWidth',1.5, 'Color','black');
hold off
ylabel('z in meters')
xlabel('time in seconds')
legend(h(2:end),'Upper Bound','Lower Bound', 'Location','northwest')
title('Reachable set of the aircraft (state z)') 
%% Plot the bounding boxes in 3D
figure(4); hold on; box on;
xLower = xStateBounds(:,1);
yLower = yStateBounds(:,1);
zLower = zStateBounds(:,1);
xUpper = xStateBounds(:,2);
yUpper = yStateBounds(:,2);
zUpper = zStateBounds(:,2);
% plot3(xLower, yLower, zLower, 'LineWidth',1.5, 'Color','black');
% plot3(xUpper, yUpper, zUpper, 'LineWidth',1.5, 'Color','black');

for patch_i  = 1:50:length(xStateBounds)
Xmin = xStateBounds(patch_i,1);
Xmax = xStateBounds(patch_i,2);
Ymin = yStateBounds(patch_i,1);
Ymax = yStateBounds(patch_i,2);
Zmin = zStateBounds(patch_i,1);
Zmax = zStateBounds(patch_i,2);
X = [Xmin;Xmax];
Y = [Ymin;Ymax];
Z = [Zmin;Zmax];
L = {X,Y,Z};
n = length(L);
[L{:}] = ndgrid(L{end:-1:1});
L = cat(n+1,L{:});
vertices = fliplr(reshape(L,[],n));
face = [1 2 6 5;2 4 8 6;3 4 8 7;1 5 7 3;1 2 4 3;5 7 8 6];
patch('Vertices',vertices,'Faces',face,'FaceColor',[0.78 0.78 0.78], 'FaceAlpha',0.2, 'EdgeColor','none')
grid on
view(3)
axis vis3d
end

% Plot the trajectories in 3D
for trace =1:numTrace
plot3(xState(trace,:), yState(trace,:), zState(trace,:), 'b','LineWidth',1.5);
hold on
end
ylabel('y in meters')
xlabel('x in meters')
zlabel('z in meters')
title('Reachable set of the system for the next 10 secs')
% xlim([-150 600])
% ylim([10 250])
% zlim([-200 400])

%% Plot projections 

figure(5); 
hold on; 
for patch_i  = 1:45:length(xStateBounds)
Xmin = xStateBounds(patch_i,1);
Xmax = xStateBounds(patch_i,2);
Ymin = yStateBounds(patch_i,1);
Ymax = yStateBounds(patch_i,2);
Zmin = zStateBounds(patch_i,1);
Zmax = zStateBounds(patch_i,2);
xyProject = [Xmin, Ymin, Xmax - Xmin, Ymax - Ymin];
xzProject = [Xmin, Zmin, Xmax - Xmin, Zmax - Zmin];
yzProject = [Ymin, Zmin, Ymax - Ymin, Zmax - Zmin];
rectangle('Position',xyProject,'FaceColor',[0.8 0.8 0.8], 'EdgeColor', 'none')
% rectangle('Position',[Xmin, Ymin, Xmax - Xmin, Ymax - Ymin], 'Curvature',[1,1])
end
for trace = 1:numTrace
plot(xState(trace,:), yState(trace,:), 'b')
end
grid on
ylabel('x2 in meters')
xlabel('x3 in meters')
title('Projection of reachtube on x2-x3 axis')


%% Compute the max diagonal of a cube for collision avoidance (Vectorize this) 
segment = 1:45:length(xStateBounds);

Xmin = xStateBounds(segment,1);
Xmax = xStateBounds(segment,2);
Ymin = yStateBounds(segment,1);
Ymax = yStateBounds(segment,2);
Zmin = zStateBounds(segment,1);
Zmax = zStateBounds(segment,2);

xRange = (Xmax - Xmin);
yRange = (Ymax - Ymin);
zRange = (Zmax - Zmin);
center = [Xmin + xRange/2, Ymin + yRange/2, Zmin + zRange/2]; 
radius = sqrt(xRange.^2 + yRange.^2 + zRange.^2) * 0.5; 




