 Ts = 0.005;
 A = [1, Ts, 0;
             0, 1,  Ts;
             0, 0,  0];
     % 0, 0, -G * abs(block.InputPort(1).Data)];
  B = [0;
         0;
         1.30767676];

     Q = diag([0.1792, 0.0055, 0.0028]);
    R = 1
    [K] = dlqr(A, B, Q, R)