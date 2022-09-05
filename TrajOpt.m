function error_motion_opt = TrajOpt(P,auxdata, Traj, x0_temp, u0_temp, bounds)
%%
tolerance = 0.03; % 可允許的軌跡點差點 tolerable error btw goal and result of a node
reap = 7; % 每一個動作重複執行的次數 repetation of each motion

import org.opensim.modeling.* 

% extract the nesessary auxiliary data
model    = auxdata.model ;
muscle   = model.getMuscles();
n_node   = auxdata.n_node;
n_state  = auxdata.n_state;
n_muscle = auxdata.n_muscle;
n_control = auxdata.n_control;
n_coord = auxdata.n_coord;
dc_time  = auxdata.time;

% extract the bounds
lb_a = bounds.lb_a;
ub_a = bounds.ub_a;
lb_b = bounds.lb_b;
ub_b = bounds.ub_b;

% Set model parameters to current value
for i = 1:n_muscle
    muscle.get(i-1).setMaxIsometricForce(P(i,1));
    muscle.get(i-1).setOptimalFiberLength(P(i,2));
    muscle.get(i-1).setTendonSlackLength(P(i,3));
end

options = optimset('algorithm','interior-point','TolFun',1e-4,'TolX',1e-4, ...
                   'TolCon',1e-4,'FinDiffType','forward','MaxFunEvals',1e5, ...
                   'Hessian','bfgs','display','off'); 
n_traj = size(Traj,2); %共有幾個動作number of motions
tracking_error_n = 1000*ones(1,n_traj);

for n = 1:n_traj 
    current_min = 1000; 
    for p = 1:reap %想跑幾次取平均/最小值 number of repetition
        %~~~~~~~~~~~~~~~~~~~~~~~ fmincon 1 ~~~~~~~~~~~~~~~~~~~~~~~% for the first two nodes
        X_state_opt = zeros(n_node,n_state); %pre-allocate size
        X_controls_opt = zeros(n_node,n_control); %pre-allocate size

        traj = Traj(1:2,n);
        X0_a = [x0_temp(1,:), x0_temp(2,:), u0_temp(2,:)]';
        objfun_a = @(X)TrajOpt_objA(traj,X,auxdata);
        confun_a = @(X)TrajOpt_conA(X,auxdata); 

        [Xopt,fval,exitflag,output] = fmincon(objfun_a,X0_a,[],[],[],[],lb_a,ub_a,confun_a,options);
        while exitflag == -2
            X0_a = [random('normal',x0_temp(1,1:2*n_coord),0.5), random('uniform',0.011,0.5,1,n_state-2*n_coord),...
                    random('normal',x0_temp(2,1:2*n_coord),0.5), random('uniform',0.011,0.5,1,n_state-2*n_coord),...
                    random('uniform',0.011,0.5,1,n_control)]';
            [Xopt,fval,exitflag,output] = fmincon(objfun_a,X0_a,[],[],[],[],lb_a,ub_a,confun_a,options);
        end
        X_state_opt(1,:) = Xopt(1:n_state)';
        X_state_opt(2,:) = Xopt(n_state+1:2*n_state)';
        X_controls_opt(1,:) = Xopt(2*n_state+1:end)';
        X_controls_opt(2,:) = Xopt(2*n_state+1:end)';

        %~~~~~~~~~~~~~~~~~~~~~~~ fmincon 2~N ~~~~~~~~~~~~~~~~~~~~~~~% for the rest nodes
        for i = 3:n_node
            traj = Traj(i,n);
            X0_b = [x0_temp(i,:), u0_temp(i,:)]';
            last_s = X_state_opt(i-1,:);
            objfun_b = @(X)TrajOpt_objB(traj,X,auxdata);
            confun_b = @(X)TrajOpt_conB(X,auxdata,last_s,dc_time(i)); 
            [Xopt,fval,exitflag,output]  = fmincon(objfun_b,X0_b,[],[],[],[],lb_b,ub_b,confun_b,options);
            while exitflag == -2 
                X0_b = [random('normal',x0_temp(i,1:2*n_coord),0.5), random('uniform',0.011,0.5,1,n_state-2*n_coord),...
                        random('uniform',0.011,0.5,1,n_control)]';
                [Xopt,fval,exitflag,output] = fmincon(objfun_b,X0_b,[],[],[],[],lb_b,ub_b,confun_b,options);
            end
            X_state_opt(i,:) = Xopt(1:n_state)';
            X_controls_opt(i,:) = Xopt(n_state+1:end)';
            
            if abs( X_state_opt(i,1)- Traj(i,n)) > tolerance
                break %有任一點距離過遠就停止這次條軌跡最佳化 this Traj Opt stop if there's any result node too far from the goal
            end
        end
        
        % 計算軌跡和量測軌跡的差
        temp = evaluateTracking( Traj(:,n), X_state_opt(:,1), tolerance )/n_node ;
        % 只打算留最小誤差
        if temp < current_min
            current_min = temp;
            result_traj(:,n) = X_state_opt(:,1);
        end      
    end 

    tracking_error_n(1,n) = current_min;
%     plot(dc_time,Traj(:,1),'-',dc_time, result_traj,'*--'); %hold on;
%     title('optimal parameters'); xlabel('simulation time (s)'); ylabel('displacemtent (m)');
%     legend({'goal trajectory','optimal trajectory'},'location','best');

end

error_motion_opt = tracking_error_n(1,:);

end
