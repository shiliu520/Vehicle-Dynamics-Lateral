function delta = SteerControl_preview(input,t)
    % 当前状态
    [X, Y, PSI, VT, ALPHAT, dPSI] = deal(input(1), input(2), input(3), input(4), input(5), input(6));
    vx = VT * cos(ALPHAT);
    vy = VT * sin(ALPHAT);

    % 全局路径
    path_s = evalin('base', 's');
    path_d = evalin('base', 'd');
    heading_angle = evalin('base', 'heading_angle');
    curvature = evalin('base', 'curvature');
    
    % [xr, yr, thetar, kappar] = findNearestPoint(path_s, path_d, heading_angle, curvature, X, Y);
    % [kr, err] = error_calc(X, Y, PSI, vx, vy, dPSI, path_s, path_d, heading_angle, curvature);
    L = 10;
    [e_d, e_psi, kappa] = error_calc_preview(X, Y, PSI, L, path_s, path_d, heading_angle, curvature);
    
    % 车辆参数
    System = evalin('base', 'System');
    % cf = -3.0054e+04;
    % cr = -2.8930e+04;
    cf = -4.6167e+04;
    cr = -3.9627e+04;
    % cf = -4.0e+04;
    % cr = -4.0e+04;
    m = System.mT;
    a = System.a;
    b = System.b;
    Iz = System.IT;
    
    Caf = -cf;
    Car = -cr;
    A11 = [-(Caf+Car)/(m*vx), (b*Car-a*Caf)/(m*vx)-vx;
        (b*Car-a*Caf)/(Iz*vx), -(a^2*Caf+b^2*Car)/(Iz*vx)];
    A12 = zeros(2, 2);
    A21 = [-1, -L;
           0, -1];
    A22 = [0, vx;
           0, 0];
    B11 = [Caf/m; a*Caf/Iz];
    B21 = zeros(2, 1);
    A = [A11, A12;
        A21, A22];
    B1 = [B11;
        B21];
    Q=1*eye(4);
    R=10;
    k=lqr(A,B1,Q,R);
    
    k1 = k(1);
    k2 = k(2);
    k4 = k(4);
    x = [vy; dPSI; e_d; e_psi];
    delta_ff = kappa * (a + b) + kappa * k4 * (b + L) + kappa * vx * (k2 + b * k1) + kappa * m * vx^2 / (Caf * Car * (a + b)) * (-a *Caf * (1 + vx * k1 + k4) + b * Car) ;
    angle=-k*x+delta_ff;
    delta = angle;
