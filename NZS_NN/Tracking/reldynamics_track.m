function statedot = reldynamics(t, state, params)

%% Unpack State and Control
    x1 = state(1);
    x2 = state(2);
    x3 = state(3);
    x4 = state(4);
    x5 = state(5);
    x6 = state(6);
    x7 = state(7);
    x8 = state(8);
    x9 = state(9);

    x = [x1, x2, x3, x4, x5, x6, x7, x8, x9]'; % state vector
    
    W1 = state(10:54);  % Player 1 Critic Weights
    W2 = state(55:99); % Player 1 Actor Weights
    W3 = state(100:144); % Player 2 Critic Weights
    W4 = state(145:189); % Player 2 Actor Weights
%% Load Parameters
    s = params.s;
    c = params.c;
    k = params.k;
    w = params.w;
   

% Matrix Definitions
    f = [0, 0, 0, 1, 0, 0, 0, 0, 0;
         0, 0, 0, 0, 1, 0, 0, 0, 0;
         0, 0, 0, 0, 0, 1, 0, 0, 0;
         (5*c^2 - 2)*(w^2), 0, 0, 0, 2*w*c, 0, 0, 0, 0;
         0, 0, 0, 2*w*c, 0, 0, 0, 0, 0;
         0, 0, -(k^2), 0, 0, 0, 0, 0, 0;
         -1, 0, 0, 0, 0, 0, 0, 0, 0;
         0, -1, 0, 0, 0, 0, 0, 0, 0;
         0, 0, -1, 0, 0, 0, 0, 0, 0];
     
    r = [zeros(6, 1); 30; 20; 0];
    % fx = f*x + r; 

    gm = [0, 0, 0;
          0, 0, 0;
          0, 0, 0;
          1, 0, 0;
          0, 1, 0;
          0, 0, 1;
          0, 0, 0;
          0, 0, 0;
          0, 0, 0];
        
    km = [0, 0, 0;
          0, 0, 0;
          0, 0, 0;
          1, 0, 0;
          0, 1, 0;
          0, 0, 1; 
          0, 0, 0;
          0, 0, 0;
          0, 0, 0];

    grad_phi1 = [
    2*x1,    0,    0,    0,    0,    0,    0,    0,    0; % x1^2
     x2,   x1,    0,    0,    0,    0,    0,    0,    0; % x1*x2
     x3,    0,   x1,    0,    0,    0,    0,    0,    0; % x1*x3
     x4,    0,    0,   x1,    0,    0,    0,    0,    0; % x1*x4
     x5,    0,    0,    0,   x1,    0,    0,    0,    0; % x1*x5
     x6,    0,    0,    0,    0,   x1,    0,    0,    0; % x1*x6
     x7,    0,    0,    0,    0,    0,   x1,    0,    0; % x1*x7
     x8,    0,    0,    0,    0,    0,    0,   x1,    0; % x1*x8
     x9,    0,    0,    0,    0,    0,    0,    0,   x1; % x1*x9
      0,  2*x2,    0,    0,    0,    0,    0,    0,    0; % x2^2
      0,   x3,   x2,    0,    0,    0,    0,    0,    0; % x2*x3
      0,   x4,    0,   x2,    0,    0,    0,    0,    0; % x2*x4
      0,   x5,    0,    0,   x2,    0,    0,    0,    0; % x2*x5
      0,   x6,    0,    0,    0,   x2,    0,    0,    0; % x2*x6
      0,   x7,    0,    0,    0,    0,   x2,    0,    0; % x2*x7
      0,   x8,    0,    0,    0,    0,    0,   x2,    0; % x2*x8
      0,   x9,    0,    0,    0,    0,    0,    0,   x2; % x2*x9
      0,    0,  2*x3,    0,    0,    0,    0,    0,    0; % x3^2
      0,    0,   x4,   x3,    0,    0,    0,    0,    0; % x3*x4
      0,    0,   x5,    0,   x3,    0,    0,    0,    0; % x3*x5
      0,    0,   x6,    0,    0,   x3,    0,    0,    0; % x3*x6
      0,    0,   x7,    0,    0,    0,   x3,    0,    0; % x3*x7
      0,    0,   x8,    0,    0,    0,    0,   x3,    0; % x3*x8
      0,    0,   x9,    0,    0,    0,    0,    0,   x3; % x3*x9
      0,    0,    0,  2*x4,    0,    0,    0,    0,    0; % x4^2
      0,    0,    0,   x5,   x4,    0,    0,    0,    0; % x4*x5
      0,    0,    0,   x6,    0,   x4,    0,    0,    0; % x4*x6
      0,    0,    0,   x7,    0,    0,   x4,    0,    0; % x4*x7
      0,    0,    0,   x8,    0,    0,    0,   x4,    0; % x4*x8
      0,    0,    0,   x9,    0,    0,    0,    0,   x4; % x4*x9
      0,    0,    0,    0,  2*x5,    0,    0,    0,    0; % x5^2
      0,    0,    0,    0,   x6,   x5,    0,    0,    0; % x5*x6
      0,    0,    0,    0,   x7,    0,   x5,    0,    0; % x5*x7
      0,    0,    0,    0,   x8,    0,    0,   x5,    0; % x5*x8
      0,    0,    0,    0,   x9,    0,    0,    0,   x5; % x5*x9
      0,    0,    0,    0,    0,  2*x6,    0,    0,    0; % x6^2
      0,    0,    0,    0,    0,   x7,   x6,    0,    0; % x6*x7
      0,    0,    0,    0,    0,   x8,    0,   x6,    0; % x6*x8
      0,    0,    0,    0,    0,   x9,    0,    0,   x6; % x6*x9
      0,    0,    0,    0,    0,    0,  2*x7,    0,    0; % x7^2
      0,    0,    0,    0,    0,    0,   x8,   x7,    0; % x7*x8
      0,    0,    0,    0,    0,    0,   x9,    0,   x7; % x7*x9
      0,    0,    0,    0,    0,    0,    0,  2*x8,    0; % x8^2
      0,    0,    0,    0,    0,    0,    0,   x9,   x8; % x8*x9
      0,    0,    0,    0,    0,    0,    0,    0,  2*x9]; % x9^2
    
    
    grad_phi2 = grad_phi1;
    
    Q2 = 100*diag([0, 0, 0, 0, 0, 0, 1, 1, 1]);
    R22 = eye(3);
    R21 = eye(3);
    Q1 = 2*Q2;
    R11 = 2*R22;
    R12 = 2*R21;
    F1 = 10*eye(45);
    F2 = 10*eye(45);
    F3 = 10*eye(45);
    F4 = 10*eye(45);
    a1 = 10;
    a2 = 10;
    a3 = 10;
    a4 = 10;

    u3 = -(1/2) * inv(R11) * gm' * grad_phi1' * W3;
    d4 = -(1/2) * inv(R22) * km' * grad_phi2' * W4;

    %% Weight Updates
    
    sig3 =  grad_phi1 * (f*x + gm * u3 + km * d4);
    sig4 =  grad_phi1 * (f*x + gm * u3 + km * d4);


    D1b = grad_phi1 * gm * inv(R11) * gm' * grad_phi1';
    D2b = grad_phi2 * km * inv(R22) * km' * grad_phi2';
    
    m1 = sig3./(sig3'*sig3+1)^2;
    m2 = sig4./(sig4'*sig4+1)^2;


    
    W1dot = -a1 * (sig3./(sig3'*sig3+1)^2)*(W1'*sig3 + norm(x)^2 + u3'*R11*u3 + d4'*R12*d4);
    W2dot = -a2 * (sig4./(sig4'*sig4+1)^2)*(W2'*sig4 + norm(x)^2 + u3'*R21*u3 + d4'*R21*d4);

    W3dot = -a3 * ((F2*W3 - F1*(W1)) - (1/4)*(grad_phi1*gm*inv(R11')*R21*inv(R11)*gm'*grad_phi1'*W3*m2'*W2 + D1b*W3*m1'*W1));
    W4dot = -a4 * ((F4*W4 - F3*(W2)) - (1/4)*(grad_phi2*km*inv(R22')*R12*inv(R22)*km'*grad_phi2'*W4*m1'*W1 + D2b*W4*m2'*W2));
    
    epsilon = @(t)  exp(-0.007 * t) * 100 * ...
               (sin(100*t)^2 * cos(t) + sin(300 * t)^2 * cos(63 * t) + ...
                sin(-1.2 * t)^2 * cos(0.5 * t) + sin(t)^5 + ...
                sin(1.12 * t)^2 + cos(2.4 * t) * sin(2.4 * t)^3);
    
    if t< 250
        unew = u3 + epsilon(t);
        dnew = d4 + epsilon(t);
    else
        unew = u3;
        dnew = d4;
    end
    xdot = f*x + gm*unew + km*dnew;
    % xdot(7) = 10 - x1;
    % xdot(8) = 10 - x2;
    % xdot(9) = 10 - x3;
    statedot = [xdot; W1dot; W2dot; W3dot; W4dot];
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