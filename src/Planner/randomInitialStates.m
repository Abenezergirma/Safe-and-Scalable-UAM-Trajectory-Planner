function initialStates = scenarioGenerator(N)
% Things to add on the scenario 
% accept the number of initial states with the number of states 
% name few scenarios and use varargin


% N - Number Of vertiports
points = linspace(0, 2*pi, N*10);
radius = 15000;
x = radius*cos(points);
X = x(1:10:end)';
y = radius*sin(points);
Y = y(1:10:end)';
z = repmat(1000,length(x),1)';
Z = z(1:10:end)';

psi = 2*pi*rand(1,length(X))';
gamma = zeros(length(X),1);
alpha_ = zeros(length(X),1); 
phi = zeros(length(X),1);
V =  repmat(24.01,length(X), 1) ;
initialStates = [X, Y, Z, psi, gamma,alpha_, phi, V];

end

% figure(1)
% plot3(x, y, z)
% hold on
% plot3([zeros(1,N); x(1:10:end)], [zeros(1,N); y(1:10:end)], [1000*ones(1,N); z(1:10:end)])
% hold off
% axis equal


% hardDeck = 100;
% lowAltitude = hardDeck + 200;
% highAltitude = lowAltitude +300;
% 
% uppper = [10000, 10000, highAltitude];
% lower = [100, 100, lowAltitude];
% 
% position = lower + (uppper-lower).*rand(1,1);
% 
% [x,y,z] = deal(position(1), position(2), position(3));



