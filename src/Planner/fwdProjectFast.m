function futureTraj = fwdProjectFast(ownship, actions, limits, timestep, numSteps)
% A function that returns a set of next states with state constraints
% applied 

%Note: this function is vectorzed to accept an array of control actions


L = 0.9;
g = 9.81;
% aircraftParamters;

currentState = ownship.currentStates;

[x, y, z, psi, gamma, alpha_, phi,V] = deal(currentState(:,1),currentState(:,2),...
    currentState(:,3),currentState(:,4),currentState(:,5),currentState(:,6),currentState(:,7),currentState(:,8));

[rollRate, alphaRate, n_x] = deal(actions(:,1), actions(:,2), actions(:,3));

[VMin, VMax, alphaMin, alphaMax, psidotMin, psidotMax, phiMin, phiMax, gammaMin, gammaMax] = deal(limits(1),...
    limits(2), limits(3), limits(4), limits(5), limits(6), limits(7), limits(8), limits(9),limits(10));

futureTraj = zeros( numSteps, length(actions(:,1)), length(currentState(1,:))); %12 is num of next state %Preallocated memory for speed

for i = 1:numSteps

    %Update angle of attack
    alpha_ = alpha_ + alphaRate * timestep; %Note "alpha" is a builtin fun in matlab 
    alpha_ = min( max(alpha_, alphaMin), alphaMax); % Enforce constraint 

    %Update the bank angle 
    phi = phi + rollRate * timestep;
    phi = wrapToPi(phi); %Wrap phi so that it is between -pi and pi
    phi = min( max(phi, phiMin), phiMax);  % Enforce constraint 


    %Compute change in speed due to n_x
    vDot = g * (n_x .* cos(alpha_) - sin(gamma));

    n_f = n_x .* sin(alpha_) + L;

    %Compute the flight path angle due to n_f
    gammaDot = (g ./ V) .* (n_f .* cos(phi) - cos(gamma));


    %compute the change in turn rate due to bank angle
    psiDot = (g ./ (V .* cos(gamma))) .* n_f .* sin(phi);
    psiDot = min( max(psiDot, psidotMin), psidotMax);  % Enforce constraint

    %Update the speed
    V = V + vDot * timestep;
    V = min( max(V, VMin), VMax);  % Enforce constraint

    %Update the turn angle 
    psi = psi + (psiDot*timestep);
    psi = wrapToPi(psi);

    %Update the flight path angle 
    gamma = gamma + gammaDot*timestep;
    gamma = wrapToPi(gamma); %Wrap gamma so that it is between -pi and pi
    gamma = min( max(gamma, gammaMin), gammaMax);  % Enforce constraint

    %Compute the pos updates
    xDot = V .* cos(gamma) .* cos(psi);
    yDot = V .* cos(gamma) .* sin(psi);
    zDot = V .* sin(gamma);

    %update the pos
    x = x + (xDot * timestep);
    y = y + (yDot * timestep);
    z = z + (zDot * timestep);
    
    nextState = [x, y, z, psi, gamma,alpha_, phi, V];
    futureTraj(i,:,:) = nextState;
end


end