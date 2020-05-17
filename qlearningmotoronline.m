%% Q-Learning Motor Online Training


%% Define Parameters

learningRate = 0.5;
discountRate = 0.3;
episodeNumber = 500;
simulationTime = 200;
epsilon = 0.95;
startPt = [0 200 400 600 800]; % Motor speed at initial
epsilonDecay = 0.05;
setPoint = 500;
integrationTime = 2;
integrationSteps = 0.05;

%% System Dynamics
% State Space Matrix
% Xdot = AX + BU \
% Y = CX
A = -6.792; 
B = 139.8862;
C = 1;

%% Define Action, State, and Rewards
% The controller will act as a bang-bang controller
states = [0 : 1000]'; % Env State
idxSet = find(states == setPoint);
R = -1 * ones(1, length(states));
R(idxSet) = 10;
action = [0 20 40 60 80 100]; % set the PWM dutycycle 
QTable = repmat(states, [1 length(action)]);

%% Q Learning

for x = 1 : episodeNumber
    
    startState = find(QTable == startPt(randi(length(startPt), 1));
    startState = startState(1);
    
    for y = 0 : simulationTime
        
        if rand() < epsilon % do a random action
            actIdx = randi(length(action), 1);
        else
            [~, actIdx] = max(QTable(startState, :));
        end
        
        doAct = action(actIdx);
        
        % System Dynamics Calculation
        
        % 4th order runge kutta
        for z = 1 : integrationTime
            
            k1 = motor(A, doAct);
            k2 = motor(A + integrationSteps / 2 * k1, T);
            k3 = motor(A + integrationSteps / 2 * k2, T);
            k4 = motor(A + integrationSteps * k3, T);
            
        end
        
        epsilon = epsilon * epsilonDecay; % Epsilon update
    
    end
    
end


%% Dynamics 
function out = motor(currentState, ctrlInput)
    out = currentState + ctrlInput;
end
  