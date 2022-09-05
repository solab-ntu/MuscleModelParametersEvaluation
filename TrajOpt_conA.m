function [c,ceq] = TrajOpt_conA(X,auxdata)
c = [];
% Import the OpenSim modeling classes
import org.opensim.modeling.*
% extract the nesessary auxiliary data
h         = auxdata.h;
osimModel = auxdata.model;
dc_time   = auxdata.time;
n_state   = auxdata.n_state;

% Check to see if model state is initialized by checking size
if(osimModel.getWorkingState().getNY() == 0)
   osimState = osimModel.initSystem();
else
   osimState = osimModel.updWorkingState(); 
end

% extract the states and controls by node (time step) from X
states = [X(1:n_state), X(n_state+1:2*n_state)]; % [state#1, state#2]
controls = X(2*n_state+1:end);

% Compute the constraint violation using backward Euler method
states_dot = (states(:,2) - states(:,1))/h;
x_dot = computeOpenSimModelXdot(states(:,2),controls(:),dc_time(2),osimModel,osimState);

% Evaluate the constraint violoation
ceq = states_dot - x_dot ;

end

