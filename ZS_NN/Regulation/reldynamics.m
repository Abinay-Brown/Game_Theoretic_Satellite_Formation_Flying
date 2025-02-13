function statedot = reldynamics(t, state, params)

%% Unpack State and Control
    x1 = state(1);
    x2 = state(2);
    x3 = state(3);
    
    x4 = state(4);
    x5 = state(5);
    x6 = state(6);
    
    x = [x1, x2, x3, x4, x5, x6]'; % state vector
    
    W1 = state(7:27);  % Critic Weights
    W2 = state(28:48); % Actor Weights
    W3 = state(49:69); % Disturbance Weights

%% Load Parameters
    s = params.s;
    c = params.c;
    k = params.k;
    w = params.w;
    gamma = params.gamma;

% Matrix Definitions
    f = [0, 0, 0, 1, 0, 0;
         0, 0, 0, 0, 1, 0;
         0, 0, 0, 0, 0, 1;
         (5*c^2 - 2)*(w^2), 0, 0, 0, 2*w*c, 0;
         0, 0, 0, 2*w*c, 0, 0;
         0, 0, -(k^2), 0, 0, 0];
        
    gm = [0, 0, 0;
          0, 0, 0;
          0, 0, 0;
          1, 0, 0;
          0, 1, 0;
          0, 0, 1];
        
    km = [0, 0, 0;
          0, 0, 0;
          0, 0, 0;
          1, 0, 0;
          0, 1, 0;
          0, 0, 1];

    grad_phi1 = [2*x1,    0,    0,   0,     0,   0;
                   x2,   x1,    0,    0,    0,   0;
                   x3,    0,   x1,    0,    0,   0;
                   x4,    0,    0,   x1,    0,   0;
                   x5,    0,    0,    0,   x1,   0;
                   x6,    0,    0,    0,    0,  x1;
                    0, 2*x2,    0,    0,    0,   0;
                    0,   x3,   x2,    0,    0,   0;
                    0,   x4,    0,   x2,    0,   0;
                    0,   x5,    0,    0,   x2,   0;
                    0,   x6,    0,    0,    0,  x2;
                    0,    0, 2*x3,    0,    0,   0;
                    0,    0,   x4,   x3,    0,   0;
                    0,    0,   x5,    0,   x3,   0;
                    0,    0,   x6,    0,    0,  x3;
                    0,    0,    0, 2*x4,    0,   0;
                    0,    0,    0,   x5,   x4,   0;
                    0,    0,    0,   x6,    0,  x4;
                    0,    0,    0,    0, 2*x5,   0;
                    0,    0,    0,    0,   x6,  x5;
                    0,    0,    0,    0,   0, 2*x6];
    Q = eye(6);
    R = eye(3);
    % F1 = 10*ones(21, 1);
    % F2 = 10*ones(21, 1);
    % F3 = 10*ones(21, 1);
    % F4 = 10*ones(21, 1);
    F1 = 10*eye(21);
    F2 = 10*eye(21);
    F3 = 10*eye(21);
    F4 = 10*eye(21);
    a1 = 10;
    a2 = 10;
    a3 = 10;

    u = -0.5*inv(R)*gm'*grad_phi1'*W2;
    d = (1/(2*gamma^2))*km'*grad_phi1'*W3;


    %% Weight Updates
    sig2 = grad_phi1 * (f*x + gm*u + km*d); 
    
    D1b = grad_phi1 * gm * inv(R) * gm' * grad_phi1';
    E1  = grad_phi1 * km * km' * grad_phi1';
    m = sig2/(sig2'*sig2 + 1)^2;
    sig2b = sig2/(sig2'*sig2 + 1);
    % disp(size(sig2b));
    % disp(size(F2*sig2b));
    W1dot = -a1*m*(sig2'*W1 + norm(x)^2 - (gamma^2)*norm(d)^2 + u'*R*u);
    W2dot = -a2*((F2*W2 - F2*W1) - 0.25*(D1b*W2*m'*W1));
    W3dot = -a3*((F4*W3 - F3*W1) + (1/(4*gamma^2))*(E1*W3*m'*W1));
    
    epsilon = @(t)  exp(-0.007 * t) * 100 * ...
               (sin(t)^2 * cos(t) + sin(2 * t)^2 * cos(0.1 * t) + ...
                sin(-1.2 * t)^2 * cos(0.5 * t) + sin(t)^5 + ...
                sin(1.12 * t)^2 + cos(2.4 * t) * sin(2.4 * t)^3);
    
    if t< 800
        unew = u + epsilon(t);
        dnew = d + epsilon(t);
    else
        unew = u;
        dnew = d;
    end
    xdot = f*x + gm*unew + km*dnew;

    statedot = [xdot; W1dot; W2dot; W3dot];
    %% Parameters
    
    % 
    % u = -K*state;
    % ux = u(1);
    % uy = u(2);
    % uz = u(3);
    
    %% Sedgwick-Schweighart Equations
    % xdd = 2*w*c*x5 + (5*c^2 - 2)*(w^2)*x1 + ux;
    % ydd = 2*w*c*x4 + uy;
    % zdd = -(k^2)*x3 + uz;

    %% Clohessy Wiltshire Equations
    % xdd = 3*(w^2)*x1 + 2*w*x5 + ux;
    % ydd = -2*w*x4 + uy;
    % zdd = -(w^2)*x3 + uz;
    
    % xdot = [x4; x5; x6; xdd; ydd; zdd];
end