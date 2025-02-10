function delta = SteerControl(input,t)
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
    [kr, err] = error_calc(X, Y, PSI, vx, vy, dPSI, path_s, path_d, heading_angle, curvature);
    % [e_d, e_psi] = error_calc_preview(0, -36, pi / 9, 10, path_s, path_d, heading_angle);
    
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
    
        A=[0,1,0,0;
        0,(cf+cr)/(m*vx),-(cf+cr)/m,(a*cf-b*cr)/(m*vx);
        0,0,0,1;
        0,(a*cf-b*cr)/(Iz*vx),-(a*cf-b*cr)/Iz,(a*a*cf+b*b*cr)/(Iz*vx)];
    B=[0;
        -cf/m;
        0;
        -a*cf/Iz];
    Q=1*eye(4);
    R=10;
    k=lqr(A,B,Q,R);
    
    forword_angle=kr*(a+b-b*k(3)-(m*vx*vx/(a+b))*((b/cf)+(a/cr)*k(3)-(a/cr)));
    angle=-k*err+forword_angle;
    delta = angle;
