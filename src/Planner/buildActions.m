function actions = buildActions()
% Builds an array that stores the action space of each team
% Fixedwing -- 
rollRates  = [-0.34896, -0.28336, -0.22096, -0.16158, -0.10510, -0.05136,...
       -0.00023, 0.00000, 0.00023, 0.05136, 0.10510, 0.16158, 0.22096,...
       0.28336, 0.34896];

alphaRates = rollRates; %force vector in g's through nose of airplane

n_xs = -2:1:3; %check this vector from the paper 

% construct the joint action space that comprises the three action spaces 
actions = {rollRates, alphaRates, n_xs};
actionsD = actions;
[actionsD{:}] = ndgrid(actions{:});
teamActions = cell2mat(cellfun(@(m)m(:),actionsD,'uni',0));

actions = teamActions; 

end