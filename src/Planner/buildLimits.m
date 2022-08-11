function limits = buildLimits()
% Builds an array that stores the limits on the vehicles 
parameters;

% All teams have same limits
VMin = 0.07 * MACH; % m/s
VMax = 0.2 * MACH; % m/s
psidotMin = - 30 * DEG2RAD; % rad/s
psidotMax =   30 * DEG2RAD; % rad/s
alphaMin = - 5 * DEG2RAD; % rad
alphaMax =  15 * DEG2RAD; % rad
phiMin = -15 * DEG2RAD ; % rad
phiMax =  15 * DEG2RAD;  % rad
gammaMin = -15 * DEG2RAD;  % rad
gammaMax =  15 * DEG2RAD;  % rad
loadMax = 9; % g's

teamLimits = [VMin, VMax, alphaMin, alphaMax, psidotMin, psidotMax, phiMin, phiMax, gammaMin, gammaMax, loadMax];

limits = teamLimits; 

end