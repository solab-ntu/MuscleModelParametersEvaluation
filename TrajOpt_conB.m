function [c,ceq] = TrajOpt_conB(X,auxdata,last_s,t)
c = [];
% Import the OpenSim modeling classes
import org.opensim.modeling.*
% extract the nesessary auxiliary data
h         = auxdata.h;
osimModel = auxdata.model;
n_state   = auxdata.n_state;

% Check to see if model state is initialized by checking size
if(osimModel.getWorkingState().getNY() == 0)
   osimState = osimModel.initSystem();
else
   osimState = osimModel.updWorkingState(); 
end

% extract the states and controls by node (time step) from X
states = X(1:n_state); 
controls = X(n_state+1:end);

% Compute the constraint violation using backward Euler method
states_dot = (states - last_s')/h;
x_dot = computeOpenSimModelXdot(states, controls,t,osimModel,osimState);

% Evaluate the constraint violoation
ceq = states_dot - x_dot ;
end

