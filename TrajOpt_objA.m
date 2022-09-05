function f = TrajOpt_objA(traj,X,auxdata)
% extract the nesessary auxiliary data
n_state       = auxdata.n_state;
stateIdx_of_coord   = 1; %狀態變數中coordinate value的index

% extract the states and controls by node (time step) from X
states = [X(1:n_state), X(n_state+1:2*n_state)]; % [state#1, state#2]
sim_coordinates = states(stateIdx_of_coord,:);

% calculate J_track 
J_track = sum( abs((traj(2,:)- sim_coordinates(:,2)')) + abs((traj(1,:)-sim_coordinates(:,1)')));%使距離最短

f = 1e5*J_track;
 end
