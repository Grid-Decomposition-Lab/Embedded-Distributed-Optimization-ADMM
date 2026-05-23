clear
close all
clc
%% System Configuration
T = 12;                         % Time horizon (hours)
N = 3;                          % Number of Energy Community Participants (ECPs), ECP 2,4,6

% define parameters
CAP_S    = 15;                  % Storage capacity
P_L_fix  = [0.8 1   1.4;...     % Fixed baseload
            0.7 0.9 0.9;...
            0.7 0.7 0.6;...
            0.5 0.5 0.5;...
            1 0.6   1  ;...
            1.4 1   1.4;...
            1.1 1.5 0.9;...
            1.6 1.2 1.2;...
            1.6 1   1.7;...
            2.5 1.3 1.7;...
            1.8 1.4 2.;...
            1.1 2.4 1.8];
P_PV     = [0 0  0;...           % PV generation profiles
            0 0  0;...
            0 0  0;...          
            1 2  2.2;...
            3 8  12;...
            8 12 16;...
            9 14 18;...
            4 15 15;...
            2 0  3;...
            3 10 13;...
            1 5  2;...
            0 0  0];

E_flex     = [2.2; 4.31; 3.84];   % Total daily flexible energy requirement
P_flex_max = 1.5;                 % Max flexible load ramping
p_G_max    = 20;                  % Grid connection limit
P_S_max    = 5;                   % Max storage power
P_EV_max   = 5;                   % Max EV charging power
T_EV       = [1; 3; 5];           % EV departure times
X_EV_tg    = [0.7; 0.8; 0.9];     % EV target SoC at departure

C_buy    = [0.53;0.52;0.56;0.57;0.6;0.61;0.54;0.46;0.49;0.4;0.45;0.48];
C_sell   = 0.08;                  % Fixed feed-in tariff (sell price)

% Efficiency and SoC Boundaries
Gam_ch   = 0.95; Gam_dch  = 0.95;                % Charge/Discharge efficiency
X_0      = 0.4 * CAP_S;                          % Initial BESS SoC
X_0_EV   = [0; 0.4 * CAP_S; 0.4 * CAP_S];        % Initial EV SoC
X_S_min  = 0.2 * CAP_S; X_S_max  = CAP_S;        % BESS SoC limits
X_EV_min = 0.2 * CAP_S; X_EV_max = CAP_S;        % EV SoC limits

%% Local Model Construction (ECP Loop)
ECP_Mats = cell(N, 1);
ops           = sdpsettings('solver','gurobi','verbose', 0);
rho           = 0.3;
for i = 1:N
    % define decision variables using YALMIP
    p_G_out  = sdpvar(T, 1);   % Grid feed-in power
    p_S_ch   = sdpvar(T, 1);   % Storage charging power
    p_S_dch  = sdpvar(T, 1);   % Storage discharging power
    p_L_flex = sdpvar(T, 1);   % Flexible load demand 
    p_EV     = sdpvar(T, 1);   % EV charging power         
    P_L_fix_i = P_L_fix(:, i);
    P_PV_i    = P_PV(:, i);
    
    % Nodal Power Balance Equation
    p_G_in = p_G_out + P_L_fix_i - P_PV_i - p_S_dch + p_S_ch + p_L_flex + p_EV;
    
    % BESS State of Charge (SoC) Dynamics
    x_S = [];
    for t = 1:T
        if t == 1
            x_S = X_0 + (Gam_ch*p_S_ch(t) - p_S_dch(t)/Gam_dch);
        else
            x_S = [x_S; x_S(t-1) + (Gam_ch*p_S_ch(t) - p_S_dch(t)/Gam_dch)]; 
        end
    end

    % EV SoC Dynamics
    x_EV     = [];
    for t = 1:T
        if t ==1
           x_EV = X_0_EV(i) + Gam_ch*p_EV(t);
        else
           x_EV = cat(2,x_EV,x_EV(t-1) + Gam_ch*p_EV(t)); 
        end
    end
    % Local Constraints (Box and Coupling)
    consts = [p_G_in >= 0; 
              p_G_in <= p_G_max; 
              p_G_out >= 0; 
              p_G_out <= p_G_max];

    consts = [consts;
              p_S_ch >= 0;
              p_S_ch <= P_S_max; 
              p_S_dch >= 0;
              p_S_dch <= P_S_max;
              x_S     >= X_S_min;
              x_S     <= X_S_max];
    % flexible load
    consts = [consts; 
              sum(p_L_flex) >= E_flex(i);
              p_L_flex >= 0; 
              p_L_flex <= P_flex_max];
    
    % electric vehicle
    if i == 1
       consts = [consts;
                 p_EV  == 0;    % No EV for Agent 1
                ];
    else
       consts = [consts;
                 p_EV  >= 0;
                 p_EV  <= P_EV_max;
                 x_EV  >= X_EV_min;
                 x_EV  <= X_EV_max;
                 x_EV(T_EV(i)) >= X_EV_tg(i);
                ];
    end
    % KKT Reformulation for ADMM Integration
    % manager's objective: Minimize (p_in - p_out)^2 (Quadratic Target)
    J_i = sum( ((p_G_in - p_G_out)/p_G_max).^2 - (p_G_out / p_G_max) );
    model_i           = export([], J_i, ops);
    ECP_Mats{i}.Q     = 2*model_i.Q;
    ECP_Mats{i}.q_QP  = model_i.obj;

    % Monetary Target: Cost minimization (Buy - Sell)
    G_i               = sum(C_buy .* p_G_in - C_sell * p_G_out);
    model_i           = export(consts, G_i, ops);
    ECP_Mats{i}.Omega = model_i.A;
    ECP_Mats{i}.Psi   = model_i.rhs;
    ECP_Mats{i}.qi    = model_i.obj;

    % Build Augmented Matrices (Primal-Dual variables)
    [m_ineq, n_u]     = size(model_i.A);
    P_i               = blkdiag(ECP_Mats{i}.Q, zeros(m_ineq, m_ineq));
    q_i               = [ECP_Mats{i}.q_QP; zeros(m_ineq, 1)];

    % Constructing KKT/ADMM constraint matrices A*x + B*z = c
    F_i               = [zeros(n_u, n_u),   model_i.A'; 
                         model_i.obj',      model_i.rhs'];   
    r_i               = [-model_i.obj; 
                              0];
    G_i               = [-eye(n_u), zeros(n_u, m_ineq);
                         model_i.A,  zeros(m_ineq, m_ineq);
                         zeros(m_ineq, n_u), -eye(m_ineq)];   
    d_i               = [zeros(n_u, 1);
                         model_i.rhs;
                         zeros(m_ineq, 1)];
    ECP_Mats{i}.P_ref = P_i;
    ECP_Mats{i}.q_ref = q_i;
    ECP_Mats{i}.F     = F_i;
    ECP_Mats{i}.r     = r_i;
    ECP_Mats{i}.G     = G_i;
    ECP_Mats{i}.d     = d_i;

    A_admm             = [G_i; F_i];
    B_admm             = [eye(size(G_i,1)); zeros(size(F_i,1), size(G_i,1))];
    c_admm             = [d_i; r_i];
    ECP_Mats{i}.A_admm = A_admm;
    ECP_Mats{i}.B_admm = B_admm;
    ECP_Mats{i}.c_admm = c_admm;

    % Pre-calculating the Inverse Matrix for x-update (Speed Optimization)
    ECP_Mats{i}.M_admm = -inv(P_i+rho*(A_admm')*A_admm);
    % reset yalmip
    yalmip('clear'); 
end

%% Distributed ADMM Solver
iter_max = 100;
iter     = 1;
eta      = 0.1;
r_k_fast = zeros(iter_max, N); 
r_k_fast_dual = zeros(iter_max, N);
for ecp = 1:N
    % Local Matrix Retrieval
    M_admm = ECP_Mats{ecp}.M_admm;
    A_admm = ECP_Mats{ecp}.A_admm;
    c_admm = ECP_Mats{ecp}.c_admm;   
    M_hat  = rho*M_admm*(A_admm');
    G_admm = ECP_Mats{ecp}.G;
    d_admm = ECP_Mats{ecp}.d;
    q_hat  = M_admm*ECP_Mats{ecp}.q_ref-M_hat*ECP_Mats{ecp}.c_admm;
    n_total_consts = size(A_admm, 1); 
    n_ineq_consts  = size(G_admm, 1); 
    mu     = zeros(n_total_consts, iter_max);    
    mu_hat = zeros(n_total_consts, iter_max);  
    z      = zeros(n_ineq_consts, iter_max);
    z_hat  = zeros(n_ineq_consts, iter_max);
    c_k    = zeros(iter_max,1);
    a      = zeros(iter_max,1);
    a(1)   = 1;
    z_hat(:,1)  = zeros(n_ineq_consts, 1);   
    mu_hat(:,1) = zeros(n_total_consts, 1); 
    c_k(1)      = inf;  

    for iter = 1:iter_max
        % Step 1: x-update (Local Minimization)
        x = q_hat + M_hat * ([z_hat(:,iter); zeros(n_total_consts - n_ineq_consts, 1)] + mu_hat(:,iter));

        % Step 2: z-update (Proximal Operator / Projection)
        z_tem          = -G_admm*x - mu_hat(1:n_ineq_consts, iter) + d_admm;
        z_tem(z_tem<0) = 0;
        z(:, iter+1)   = z_tem;

        % Step 3: mu-update (Dual Update)
        primal_error     = A_admm*x + [z(:,iter+1); zeros(n_total_consts - n_ineq_consts, 1)] - c_admm;
        mu(:, iter+1)    = mu_hat(:, iter) + primal_error;

        % Recording Residuals
        c_k(iter+1) = (1/rho) * norm(mu(:, iter+1) - mu_hat(:, iter),2)^2 + rho* norm(z(:, iter+1)  - z_hat(:, iter),2)^2;

        if c_k(iter+1) < eta * c_k(iter)
            % no restart
            a(iter+1)          = 0.5 + sqrt(1 + 4*a(iter)^2) / 2;
            z_hat(:, iter+1)   = z(:,iter+1)  + (a(iter)-1)/a(iter+1) * (z(:,iter+1)  - z(:,iter));
            mu_hat(:, iter+1)  = mu(:,iter+1) + (a(iter)-1)/a(iter+1) * (mu(:,iter+1) - mu(:,iter));
        else
            % restart
            a(iter+1)          = 1;
            z_hat(:, iter+1)   = z(:, iter);      
            mu_hat(:, iter+1)  = mu(:, iter);    
            c_k(iter+1)        = c_k(iter)/eta;
            %z(:, iter+1)       = z(:, iter);
            %mu(:, iter+1)      = mu(:, iter);
        end
          r_k_fast(iter, ecp) = norm(primal_error);
          r_k_fast_dual(iter, ecp) = norm(rho*(A_admm')*[z(:,iter+1)-z(:,iter);zeros(n_total_consts-n_ineq_consts,1)]);

    end
end
%% --- Visualization ---
CM = {'#CD5555';'#6CA6CD';'#A2CD5A';'#EDB120'};
figure
h = plot(r_k_fast);
set(h,{'Color'}, CM(1:3));
title('Primal Residual Convergence')
xlabel('Iteration')
ylabel('||r_p||')
legend('ECP 1','ECP 2','ECP 3')
grid on
setfig

figure
h = plot(r_k_fast_dual);
set(h,{'Color'}, CM(1:3));
title('Dual Residual Convergence')
xlabel('Iteration')
ylabel('||r_d||')
legend('ECP 1','ECP 2','ECP 3')
grid on
setfig