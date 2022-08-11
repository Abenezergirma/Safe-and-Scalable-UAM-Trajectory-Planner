clear; close all; clc;

load('32MDP.mat')

%to get the array with max length
for path=1:length(droneList)
    cellstore{path} = droneList{path}.traveledPath(:,1:3);
end
maxlength = 500;%max(cellfun(@length,cellstore));

xTraj = zeros(length(droneList), maxlength);
yTraj = zeros(length(droneList), maxlength);
zTraj = zeros(length(droneList), maxlength);
bestTrajectory =  zeros(length(droneList), 10, 3, maxlength); %(agents, 10futuresteps, (x,y,z), maxstep)

for st = 1:maxlength
    plot3(squeeze(bestTrajectory(3,:,1,st)), squeeze(bestTrajectory(3,:,2,st)), squeeze(bestTrajectory(3,:,3,st)))
    hold on
end

for i = 1:length(droneList)
    % paths
    xTraj(i,1:length(droneList{i}.traveledPath(:,1))) = droneList{i}.traveledPath(:,1);
    yTraj(i,1:length(droneList{i}.traveledPath(:,2))) = droneList{i}.traveledPath(:,2)';
    zTraj(i,1:length(droneList{i}.traveledPath(:,3))) = droneList{i}.traveledPath(:,3)';
    % best trajectories
    bestTraject = droneList{i}.bestTrajectory;
    bestTraject = permute(reshape(bestTraject, 10, length(bestTraject)/10, 3), [1 3 2]);
    bestTrajectory(i,:,:,1:length(bestTraject)) = bestTraject;

    % concatenate the last parts with the last element
    xTraj(i,xTraj(i,:)==0) = droneList{i}.traveledPath(end,1);
    yTraj(i,yTraj(i,:)==0) = droneList{i}.traveledPath(end,2);
    zTraj(i,zTraj(i,:)==0) = droneList{i}.traveledPath(end,3);

%     bestTrajectory(i,squeeze(bestTrajectory(i,:,:,:)==0)) = ...
        reshape(repmat(droneList{i}.bestTrajectory(end,:), 10*(maxlength - length(droneList{i}.traveledPath(:,1))), 1), 1, []);

end


%%
% figure();
% define Path, aircraft, and future Traj handlers and plot the goal states
for vehicle = 1:length(droneList)
    aircraftHandle(vehicle) = plot3(xTraj(vehicle,1),yTraj(vehicle,1),zTraj(vehicle,1),'o','MarkerFaceColor','red');
    hold on
    pathHandle(vehicle) = plot3([xTraj(vehicle,1)], [yTraj(vehicle,1)], [zTraj(vehicle,1)], 'LineWidth',1.2,'Color','blue');


    bestTrajHandle(vehicle) = plot3([xTraj(vehicle,1)], [yTraj(vehicle,1)], [zTraj(vehicle,1)], 'LineWidth',1.5, 'Color','black');

    plot3(droneList{vehicle}.goal(1),droneList{vehicle}.goal(2),droneList{vehicle}.goal(3),'-s','LineWidth',5,'Color','black')

%     plot3(xTraj(1,1),yTraj(1,1),zTraj(1,1),'-s','LineWidth',5,'Color','black')
end

xlim([min(xTraj,[],'all') - 1000, max(xTraj,[],'all') + 5000 ])
ylim([min(yTraj,[],'all') - 1000, max(yTraj,[],'all') + 5000 ])
zlim([min(zTraj,[],'all') - 500, max(zTraj,[],'all') + 500])
ylabel('y in meters')
xlabel('x in meters')
zlabel('z in meters')
title('Navigation with '+string(totalAgents) +' agents')
view(90,0)
grid on

% get figure size
pos = get(gcf, 'Position');
width = pos(3); height = pos(4);

% preallocate data (for storing frame data)
mov = zeros(height, width, 1, length(xTraj), 'uint8');

stepCounter = title(sprintf('step = %.2f', 1));
for p = 1:maxlength
    for aircraft = 1:length(droneList)
        %display the time and NMACs also
        set(stepCounter, 'String', sprintf('step = %.2f',p));

        % plot the aircrafts
        set(aircraftHandle(aircraft), 'XData', xTraj(aircraft,p), 'YData', yTraj(aircraft,p), 'ZData', zTraj(aircraft,p))

        %plot the paths
        % replaced aircraft with 1
        pathHandle(aircraft).XData = [pathHandle(aircraft).XData, xTraj(aircraft,p)];
        pathHandle(aircraft).YData = [pathHandle(aircraft).YData, yTraj(aircraft,p)];
        pathHandle(aircraft).ZData = [pathHandle(aircraft).ZData, zTraj(aircraft,p)];

        %plot the best next trajectories
        set(bestTrajHandle(aircraft), 'XData', bestTrajectory(aircraft,:,1,p), 'YData', bestTrajectory(aircraft,:,2,p) ...
            , 'ZData', bestTrajectory(aircraft,:,3,p))

        %         drawnow
        f = getframe(gcf);

        if p == 1
            [mov(:,:,1,p), map] = rgb2ind(f.cdata, 256, 'nodither');
        else
            mov(:,:,1,p) = rgb2ind(f.cdata, map, 'nodither');
        end

    end
end

% Create animated GIF
imwrite(mov, map, 'animation.gif', 'DelayTime', 1/60, 'LoopCount', inf);

%%

% plot(droneList{9}.traveledPath(:,4))
%
%
% plot(squeeze(droneList{1}.pastControls))

