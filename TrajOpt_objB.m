function f = TrajOpt_objB(traj,X,auxdata)
% extract the nesessary auxiliary data
n_state       = auxdata.n_state;
stateIdx_of_coord   = 1; %狀態變數中coordinate value的index，若多自由度則為陣列

% extract the states and controls by node (time step) from X
states = X(1:n_state); 

sim_coordinates = states(stateIdx_of_coord,:);

% calculate J_track and J_mus
J_track = sum( abs((traj(1,:)- sim_coordinates(:,1)')));%使距離最短

f = 1e5*J_track;
end
