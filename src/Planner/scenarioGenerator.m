function initialStates = scenarioGenerator(N, varargin)
% Accept the number of initial states and the type of scenario 


%set defaults
scenario = 'circle';

% process optional inputs
arg = varargin;
if ischar(arg{1})
    scenario = arg{1};
end

switch lower(scenario)
    case {'circle'}
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

    case {'random'}
        % N - Number of random initial points

        hardDeck = 100;
        lowAltitude = hardDeck + 200;
        highAltitude = lowAltitude +300;
        
        upperX = 10000; 
        upperY = 10000;
        upperZ = highAltitude;
        lowerX = 100; 
        lowerY = 100;
        lowerZ = lowAltitude;
     
        X = lowerX + (upperX-lowerX).*rand(1,N);
        Y = lowerY + (upperY-lowerY).*rand(1,N);
        Z = lowerZ + (upperZ-lowerZ).*rand(1,N);
        psi = 2*pi*rand(1,length(X))';
        gamma = zeros(length(X),1);
        alpha_ = zeros(length(X),1);
        phi = zeros(length(X),1);
        V =  repmat(24.01,length(X), 1) ;
        initialStates = [X', Y', Z', psi, gamma,alpha_, phi, V];
     
    
    otherwise
        error('scenario building on progress')
end

end





