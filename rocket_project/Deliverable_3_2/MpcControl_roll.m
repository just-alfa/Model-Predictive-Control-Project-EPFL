classdef MpcControl_roll < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc, Ts, H)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   X(:,1)       - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   U(:,1)       - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(H/Ts); % Horizon steps
            N = N_segs + 1;      % Last index in 1-based Matlab indexing
            
            [nx, nu] = size(mpc.B);
            
            % Steady-state targets (Ignore this before Todo 3.2)
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);
            
            % Predicted state and input trajectories
            X = sdpvar(nx, N);
            U = sdpvar(nu, N-1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            obj = 0;
            con = [];

            % state constraints
            %none 

            % input constraints
            M = [1; -1];
            m = [20; 20];
            
            % matrices
            Q = 50*eye(2);
            R = 0.1;
            
            sys = LTISystem('A',mpc.A,'B',mpc.B);

            sys.x.max = [Inf;Inf];
            sys.x.min = [-Inf;-Inf];
            sys.u.min = [-20];
            sys.u.max = [20];
            sys.x.penalty = QuadFunction(Q);
            sys.u.penalty = QuadFunction(R);

            Qf = sys.LQRPenalty.weight;
            Xf = sys.LQRSet;
            
            Ff = double(Xf.A);
            ff = double(Xf.b);

            obj = 0;
            con = [];

            for i = 1:N-1
                con = [con, (X(:,i+1)) == mpc.A*(X(:,i)) + mpc.B*(U(:,i))]; % System dynamics
                con = [con, M*U(:,i) <= m]; % Input constraints
                obj = obj + (X(:,i+1)-x_ref)'*Q*(X(:,i+1)-x_ref) + (U(:,i)-u_ref)'*R*(U(:,i)-u_ref); % Cost function
            end
            con = [con, Ff*(X(:,N)-x_ref) <= ff]; % Terminal constraint
            obj = obj + (X(:,N)-x_ref)'*Qf*(X(:,N)-x_ref); % Terminal weight

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {X(:,1), x_ref, u_ref}, {U(:,1), X, U});
        end
        
        % Design a YALMIP optimizer object that takes a position reference
        % and returns a feasible steady-state state and input (xs, us)
        function target_opti = setup_steady_state_target(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs, us - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Steady-state targets
            nx = size(mpc.A, 1);
            xs = sdpvar(nx, 1);
            us = sdpvar;
            
            % Reference position (Ignore this before Todo 3.2)
            ref = sdpvar;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
            obj = 0;
            con = [xs == 0, us == 0];

            Sigma = [eye(nx)-mpc.A, -mpc.B; mpc.C, zeros(size(mpc.C,1), size(mpc.B,2))];

            Q_sigma = 0.01*eye(2);

            R_sigma = 1;

            B_Sigma = [zeros(nx,1); ref];
            
            obj = us'*R_sigma*us;

            % input constraints
            M = [1; -1];
            m = [20; 20];

            con = [Sigma*[xs;us]==B_Sigma,
                           M*us<= m];
            
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end
