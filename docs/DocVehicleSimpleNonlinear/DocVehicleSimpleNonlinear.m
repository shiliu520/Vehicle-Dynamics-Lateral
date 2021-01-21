%% Vehicle Simple Nonlinear Modeling
% This scipt derives the equations of motion from the nonlinear simple
% vehicle model.
%
% For class documentation, run:
%
% |doc VehicleDynamicsLateral.VehicleSimpleNonlinear|
%
% <html>
% <!--
% MathJax
% Source: http://docs.mathjax.org/en/latest/start.html
% -->
% <script type="text/javascript" async
% src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.5/MathJax.js?config=TeX-MML-AM_CHTML">
% </script>
% <!--
% Automatic equation numbering
% Source: http://docs.mathjax.org/en/latest/tex.html#automatic-equation-numbering
% -->
% <script type="text/x-mathjax-config">
% MathJax.Hub.Config({
% TeX: { equationNumbers: { autoNumber: "AMS" } }
% });
% </script>
% </html>
%
%% Bicycle model
% <html> 
% <p>The single track (bicycle) approximation is illustrated below</p>
% <p><img vspace="5" hspace="5" src="modelSimple.svg" alt=""> </p>
% <p>The vector basis \( \Omega_{\rm O} = \{ {\rm O} {\bf i} {\bf j} {\bf k} \}\) is fixed to the inertial reference frame. The vector basis \( \Omega_{\rm T} = \{ {\rm T} {\bf t}_x {\bf t}_y {\bf t}_z \}\) is fixed to the simple vehicle. The vector basis \(\{ {\rm F} {\bf e}_x {\bf e}_y {\bf e}_z \}\) is fixed to the front axle.</p>
% <p>The center of gravity of the vehicle is located at the point \({\rm T}\). The front and rear axles are located at the points \({\rm F}\) and \({\rm R}\), respectively. Point \({\rm O}\) is the origin. The constant \(a\) measures the distance of point \({\rm F}\) to \({\rm T}\) and \(b\) the distance of point \({\rm T}\) to \({\rm R}\). The angles \(\alpha_{\rm F}\) e \(\alpha_{\rm R}\) are the front and rear slip angles, respectively. \(\alpha_{\rm T}\) is the vehicle side slip angle and \(\psi\) is the vehicle yaw angle. \(\delta\) is the steering angle.</p>
% </html>
%
%% Initialization
% Defining Symbols (sym) and Symbolic Functions (symfun). The subscript t
% indicates time symbolic functions.
%
% See
% <https://www.mathworks.com/help/symbolic/create-symbolic-functions.html create-symbolic-functions>.
%

clear ; close all ; clc

% Generalized coordinates (symfun)
syms x_t(t) y_t(t) PSI_t(t)
% Generalized coordinates and time derivatives (sym)
syms x dx ddx y dy ddy PSI dPSI ddPSI

% Alternative velocity related variables: Side slip angle and speed
% Alternative variables (symfun)
syms ALPHAT_t(t) VT_t(t)
% Alternative variables and time derivatives (sym)
syms VT ALPHAT dVT dALPHAT

% Inputs 
syms deltaf deltar FxF FxR
% Tire lateral forces
syms FyF FyR
% Vehicle parameters 
syms a b mT IT

%% Generalized coordinates and time derivatives
% <html>
% <p>The generalized coordinates are</p>
% <p>
% \begin{eqnarray}
% \nonumber q_1 &=& x     \\
% \nonumber q_2 &=& y     \\
% \nonumber q_3 &=& \psi
% \end{eqnarray}
% </p>
% <p>where \(x\) and \(y\) are the coordinates of the CG of the vehicle.
% \(\psi\) is the yaw angle of the vehicle.</p> 
% </html>

% symfun
q_t     = [x_t y_t PSI_t];
dq_t    = diff(q_t,t);
ddq_t   = diff(diff(q_t,t));
% sym
q       = [ x   y   PSI   ];
dq      = [ dx  dy  dPSI  ];
ddq     = [ ddx ddy ddPSI ];

%% Alternative velocity related variables 
% <html>
% <p>However, in many occasions it is more convenient to use the states \(v_{\rm T}\) e \(\alpha_{\rm T}\) instead of \(\dot{x}\) e \(\dot{y}\).</p>
% <p>The equations relating this pairs of variables is</p>
% <p>
%     \begin{eqnarray}
%     \nonumber \dot{x} &=& v_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) \\
%     \dot{y} &=& v_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) \label{eq:dxdy2vtalphat}
%     \end{eqnarray}
% </p>
% </html>

altVar_t    = [ALPHAT_t(t) VT_t(t)];
altVarD1_t  = diff(altVar_t,t);
altVar      = [ALPHAT VT];
altVarD1    = [dALPHAT dVT];

%% Conversion variables
% Array to switch symfun <-> sym

symfunVariables = [q_t  dq_t    ddq_t   altVar_t    altVarD1_t  ];
symVariables    = [q    dq      ddq     altVar      altVarD1    ];

%% Position
% <html>
% <p>The position of the CG of the tractor relative to the origin \({\rm O}\) is</p>
% <p>
% \begin{equation*} {\bf p}_{{\rm T}/{\rm O}} = x \, {\bf i} + y \, {\bf j}. \end{equation*}
% </p>
% <p>Relative positions are</p>
% <p>
% \begin{eqnarray}
% \nonumber {\bf p}_{{\rm F}/{\rm T}} &=& a \cos \psi {\bf i} + a \sin \psi {\bf j}. \\
% \nonumber {\bf p}_{{\rm R}/{\rm T}} &=& - b \cos \psi {\bf i} - b \sin \psi {\bf j}. \\
% \end{eqnarray}
% </p>
% <p>Position of axles</p>
% <p>
% \begin{eqnarray}
% \nonumber {\bf p}_{{\rm F}/{\rm O}} &=& {\bf p}_{{\rm F}/{\rm T}} + {\bf p}_{{\rm T}/{\rm O}} = \left( x + a \cos \psi \right) {\bf i} + \left( y + a \sin \psi \right) {\bf j}. \\
% \nonumber {\bf p}_{{\rm R}/{\rm O}} &=& {\bf p}_{{\rm F}/{\rm T}} + {\bf p}_{{\rm T}/{\rm O}} = \left( x - b \cos \psi \right) {\bf i} + \left( y - b \sin \psi \right) {\bf j}. 
% \end{eqnarray}
% </p>
% </html>

% CG position
PTO_t = [x_t y_t 0];                                            % P_{T/O}

% Relative positions
PFT_t = [ a*cos(PSI_t)  a*sin(PSI_t) 0];                        % P_{F/T}
PRT_t = [-b*cos(PSI_t) -b*sin(PSI_t) 0];                        % P_{R/T}

% Axle position
PFO_t = PFT_t + PTO_t;                                          % P_{F/O}
PRO_t = PRT_t + PTO_t ;                                         % P_{R/O}

PFO   = subs(PFO_t,symfunVariables,symVariables);               % P_{F/O}
PRO   = subs(PRO_t,symfunVariables,symVariables);               % P_{R/O}

%% Velocity 
%
% <html>
% <p>The angular velocity is</p>
% <p>
%     \begin{equation}
%     {\bf w}_{\rm T} = \dot{\psi} {\bf k}. \label{eq:tractorvelangular}
%     \end{equation}
% </p>
% <p>CG velocity</p>
% <p>
%     \begin{equation}
%     {\bf v}_{\rm T} = \dot{x} {\bf i} + \dot{y} {\bf j} \label{eq:tractorveltranslation} 
%     \end{equation}
% </p>
% <p>Velocities of each axle are</p>
% <p>
%     \begin{eqnarray}
%     {\bf v}_{\rm F} &=& \left( \dot{x} - a \dot{\psi} \sin \psi \right) {\bf i} + \left( \dot{y} + a \dot{\psi} \cos \psi \right) {\bf j} \label{eq:velfront} \\
%     {\bf v}_{\rm R} &=& \left( \dot{x} + b \dot{\psi} \sin \psi \right) {\bf i} + \left( \dot{y} -b \dot{\psi} \cos \psi \right) {\bf j} \label{eq:velrear} 
%     \end{eqnarray}
% </p>
% </html>
%

% Angular velocity
wT_t        = [0 0 diff(PSI_t,t)];

% Velocity (T)
VTdiff_t    = diff(PTO_t,t);
VTdiff      = subs(VTdiff_t,symfunVariables,symVariables);

% Velocity - Alternative definition
dx_Alt_t    = VT_t*cos(PSI_t + ALPHAT_t);
dy_Alt_t    = VT_t*sin(PSI_t + ALPHAT_t);
dx_Alt      = subs(dx_Alt_t,symfunVariables,symVariables);
dy_Alt      = subs(dy_Alt_t,symfunVariables,symVariables);

% Acceleration - Alternative definition
ddx_Alt_t   = simplify(diff(dx_Alt_t,t));
ddy_Alt_t   = simplify(diff(dy_Alt_t,t));
ddx_Alt     = subs(ddx_Alt_t,symfunVariables,symVariables);
ddy_Alt     = subs(ddy_Alt_t,symfunVariables,symVariables);

%% Slip angles
% <html>
% <p>Using equations \ref{eq:velfront} and \ref{eq:velrear}, the slip angles can be written as</p>
% <p>
% \begin{eqnarray}
% \nonumber \alpha_{\rm F} &=& \arctan \left( \frac{\dot{y} + a \dot{\psi} \cos \psi}{ \dot{x} - a \dot{\psi} \sin \psi} \right) - \left( \delta_{\rm F} + \psi \right) \\
% \nonumber \alpha_{\rm R} &=& \arctan \left( \frac{\dot{y} - b \dot{\psi} \cos \psi}{ \dot{x} + b \dot{\psi} \sin \psi} \right) - \left( \delta_{\rm R} + \psi \right)
% \end{eqnarray}
% </p>
% <p>Using equations in \ref{eq:dxdy2vtalphat}, the slip angles become</p>
% <p>
% \begin{eqnarray}
% \alpha_{\rm F} &=& \arctan \left( \frac{v_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) + a \dot{\psi} \cos \psi}{ v_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) - a \dot{\psi} \sin \psi} \right) - \left( \delta_{\rm F} + \psi \right) \label{eq:slipangleFfull} \\
% \alpha_{\rm R} &=& \arctan \left( \frac{v_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) - b \dot{\psi} \cos \psi}{ v_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) + b \dot{\psi} \sin \psi} \right) - \left( \delta_{\rm R} + \psi \right) \label{eq:slipangleRfull}
% \end{eqnarray}
% </p>
% <p>Since the definition of slip angle does not actually depend on the orientation, the orientation may be considered zero for simplification. For the tractor, orientation is \(\psi\). Thus, (\(\psi=0\)) in \ref{eq:slipangleFfull} and \ref{eq:slipangleRfull}. Then </p>
% <p>
% \begin{eqnarray}
% \nonumber \alpha_{\rm F} &=& \arctan \left( \frac{v_{\rm T} \sin \alpha_{\rm T} + a \dot{\psi}}{ v_{\rm T} \cos \alpha_{\rm T}} \right) - \delta_{\rm F} \\
% \alpha_{\rm R} &=& \arctan \left( \frac{v_{\rm T} \sin \alpha_{\rm T} - b \dot{\psi}}{ v_{\rm T} \cos \alpha_{\rm T}} \right) - \delta_{\rm R}
% \label{eq:slipanglesnewvariables}
% \end{eqnarray}
% </p>
% </html>

VF_t    = diff(PFO_t,t);
VR_t    = diff(PRO_t,t);

VF      = formula(subs(VF_t,symfunVariables,symVariables));
VR      = formula(subs(VR_t,symfunVariables,symVariables));

VF_alt  = simplify(subs(VF,[dx dy],[dx_Alt dy_Alt]));
VR_alt  = simplify(subs(VR,[dx dy],[dx_Alt dy_Alt]));

% Slip angle definition
ALPHAF_def = atan2(VF_alt(2),VF_alt(1)) - (deltaf + PSI);
ALPHAR_def = atan2(VR_alt(2),VR_alt(1)) - (deltar + PSI);

% Slip angle simplification (PSI=0 - Orientation is irrelevant here)
% Orientation can be neglected, i.e., for the truck: Orientation=PSI=0 
ALPHAF = subs(ALPHAF_def,PSI,0);
ALPHAR = subs(ALPHAR_def,PSI,0);

%%
disp(ALPHAF)
%%
disp(ALPHAR)

%% Forces
% <html>
% <p>Force vectors at front, rear and semitrailer axles is</p>
% <p>
%     \begin{eqnarray}
%     \nonumber {\bf F}_{\rm F} &=& F_{x,{\rm F}} \, {\bf e}_x + F_{y,{\rm F}} \, {\bf e}_y \\
%     \nonumber {\bf F}_{\rm R} &=& F_{x,{\rm R}} {\bf t}_x + F_{y,{\rm R}} {\bf t}_y 
%     \end{eqnarray}
% </p>
% <p>or</p>
% <p>
%     \begin{eqnarray}
%     {\bf F}_{\rm F} &=& \left[ F_{x,{\rm F}} \cos \left( \psi + \delta_{\rm F} \right) - F_{y,{\rm F}} \sin \left( \psi + \delta_{\rm F} \right) \right] {\bf i} + \left[ F_{x,{\rm F}} \sin \left( \psi + \delta_{\rm F} \right) + F_{y,{\rm F}} \cos \left( \psi + \delta_{\rm F} \right) \right] {\bf j} \label{eq:forcefront} \\
%     {\bf F}_{\rm R} &=& \left[ F_{x,{\rm R}} \cos \left( \psi + \delta_{\rm R} \right) - F_{y,{\rm R}} \sin \left( \psi + \delta_{\rm R} \right) \right] {\bf i} + \left[ F_{x,{\rm R}} \sin \left( \psi + \delta_{\rm R} \right) + F_{y,{\rm R}} \cos \left( \psi + \delta_{\rm R} \right) \right] {\bf j}. \label{eq:forcerear} 
%     \end{eqnarray}
% </p>
% </html>

% Force vectors
FF = [FxF*cos(PSI+deltaf)-FyF*sin(PSI+deltaf)           FxF*sin(PSI+deltaf)+FyF*cos(PSI+deltaf)     0];
FR = [FxR*cos(PSI+deltar)-FyR*sin(PSI+deltar)           FxR*sin(PSI+deltar)+FyR*cos(PSI+deltar)     0];

%%
% <html>
% <p>The generalized forces are</p>
% <p>
%     \begin{equation}
%     Q_k = \sum_{j = 1} ^p {\bf F}_j \cdot \frac{\partial {\bf p}_j}{\partial q_k} \qquad \qquad \begin{array}{c} k = 1, 2, 3 \\ j = {\rm F}, {\rm R} \end{array}.
%     \end{equation}
% </p>
% <p>This is</p>
% <p>
%     \begin{eqnarray}
%     \nonumber Q_1 &=& {\bf F}_{\rm F} \cdot \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_1} + {\bf F}_{\rm R} \cdot \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_1} \\
%     \nonumber Q_2 &=& {\bf F}_{\rm F} \cdot \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_2} + {\bf F}_{\rm R} \cdot \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_2} \\
%     \nonumber Q_3 &=& {\bf F}_{\rm F} \cdot \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_3} + {\bf F}_{\rm R} \cdot \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_3} 
%     \end{eqnarray}
% </p>
% <p>Thus</p>
% <p>
%     \begin{eqnarray}
%     \nonumber \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_1} &=& \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial x} = {\bf i} \\
%     \nonumber \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_2} &=& \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial y} = {\bf j} \\
%     \nonumber \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_3} &=& \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial \psi} = - a \sin \psi {\bf i} + a \cos \psi {\bf j}
%     \end{eqnarray}
% </p>
% <p>and</p>
% <p>
%     \begin{eqnarray}
%     \nonumber \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_1} &=& \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial x} = {\bf i} \\
%     \nonumber \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_2} &=& \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial y} = {\bf j} \\
%     \nonumber \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_3} &=& \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial \psi} = b \sin \psi {\bf i} - b \cos \psi {\bf j}
%     \end{eqnarray}
% </p>
% </html>

% Generalized forces
Q1 = FF*diff(PFO,x)'    + FR*diff(PRO,x)'    ;
Q2 = FF*diff(PFO,y)'    + FR*diff(PRO,y)'    ;
Q3 = FF*diff(PFO,PSI).' + FR*diff(PRO,PSI).' ;

%%
% <html>
% <p>Substituting equations</p>
% <p>
%     \begin{eqnarray}
%     \nonumber Q_1 &=& F_{x,{\rm F}} \cos \left( \psi + \delta_{\rm F} \right) - F_{y,{\rm F}} \sin \left( \psi + \delta_{\rm F} \right) + F_{x,{\rm R}} \cos \left( \psi + \delta_{\rm R} \right) - F_{y,{\rm R}} \sin \left( \psi + \delta_{\rm R} \right) \\
%     \nonumber Q_2 &=& F_{x,{\rm F}} \sin \left( \psi + \delta_{\rm F} \right) + F_{y,{\rm F}} \cos \left( \psi + \delta_{\rm F} \right) + F_{x,{\rm R}} \sin \left( \psi + \delta_{\rm R} \right) + F_{y,{\rm R}} \cos \left( \psi + \delta_{\rm R} \right) \\
%     \nonumber Q_3 &=& F_{x,{\rm F}} a \sin(\delta_{\rm F}) + F_{y,{\rm F}} a \cos(\delta_{\rm F}) - F_{x,{\rm R}} b \sin(\delta_{\rm R}) - F_{y,{\rm R}} b \cos(\delta_{\rm R})
%     \end{eqnarray}
% </p>
% </html>

% Generalized forces array
Q = formula(simplify([Q1 ; Q2 ; Q3]));

%% Kinetic Energy
% <html>
% <p>The kinetic energy of the system is</p>
% <p>
%     \begin{equation} \label{eq:kineticenergy} T = \frac{1}{2} m_{\rm T} {\bf v}_{\rm T} \cdot {\bf v}_{\rm T} + \frac{1}{2} \left\{ {\bf w}_{\rm T} \right\}^T \left[ {\bf J}_{\rm T} \right] \left\{ {\bf w}_{\rm T} \right\}.\end{equation}
% </p>
% <p>Substituting \ref{eq:tractorvelangular} and \ref{eq:tractorveltranslation} in \ref{eq:kineticenergy} </p>
% <p>
%     \begin{equation}T = \frac{1}{2} m_{\rm T} \left( \dot{x}^2 + \dot{y}^2 \right) + \frac{1}{2} I_{\rm T} \dot{\psi}^2,\end{equation}
% </p>
% </html>

T = 1/2*mT*(VTdiff*VTdiff.') + 1/2*IT*dPSI^2;

%% Euler-Lagrange
% <html>
% <p>The Euler-Lagrange formulation for this system is</p>
% <p>
%     \begin{equation}
%     \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{q}_k} \right) - \frac{\partial T}{\partial q_k} = Q_k \qquad \qquad k = 1, 2, 3
%     \end{equation}
% </p>
% <p>where</p>
% <p>
%     \begin{eqnarray}
%     \frac{\partial T}{\partial q_1} &=& \frac{\partial T}{\partial x} = 0 \\
%     \frac{\partial T}{\partial q_2} &=& \frac{\partial T}{\partial y} = 0 \\
%     \frac{\partial T}{\partial q_3} &=& \frac{\partial T}{\partial \psi} = 0
%     \end{eqnarray}
% </p>
% </html>

dTdq1 = diff(T,x);
dTdq2 = diff(T,y);
dTdq3 = diff(T,PSI);
dTdq = [ dTdq1 ; dTdq2 ; dTdq3 ];
dTdq = subs(dTdq,symVariables,symfunVariables);

%%
% <html>
% <p>The partial derivatives are</p>
% <p>
%     \begin{eqnarray}
%     \nonumber \frac{\partial T}{\partial \dot{q}_1} &=& \frac{\partial T}{\partial \dot{x}} = m_{\rm T} \dot{x} \\
%     \nonumber \frac{\partial T}{\partial \dot{q}_2} &=& \frac{\partial T}{\partial \dot{y}} = m_{\rm T} \dot{y} \\
%     \nonumber \frac{\partial T}{\partial q_3} &=& \frac{\partial T}{\partial \dot{\psi}} = I_{\rm T} \dot{\psi} 
%     \end{eqnarray}
% </p>
% <p>Time derivatives</p>
% <p>
%     \begin{eqnarray}
%     \nonumber \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{q}_1} \right) &=& \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{x}} \right) = m_{\rm T} \ddot{x} \\
%     \nonumber \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{q}_2} \right) &=& \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{y}} \right) = m_{\rm T} \ddot{y} \\
%     \nonumber \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{q}_3} \right) &=& I_{\rm T} \ddot{\psi}
%     \end{eqnarray}
% </p>
% </html>

dTddq1 = diff(T,dx);
dTddq2 = diff(T,dy);
dTddq3 = diff(T,dPSI);
dTddq  = [ dTddq1 ; dTddq2 ; dTddq3 ];
dTddq  = subs(dTddq,symVariables,symfunVariables);
ddTddq = diff(dTddq,t);

%% State equations
% <html>
% <p>The equations of motion are</p>
% <p>
%     \begin{eqnarray}
%     \nonumber m_{\rm T} \ddot{x} &=& F_{x,{\rm F}} \cos \left( \psi + \delta_{\rm F} \right) - F_{y,{\rm F}} \sin \left( \psi + \delta_{\rm F} \right) + F_{x,{\rm R}} \cos \left( \psi + \delta_{\rm R} \right) - F_{y,{\rm R}} \sin \left( \psi + \delta_{\rm R} \right) \\
%     \nonumber m_{\rm T} \ddot{y} &=& F_{x,{\rm F}} \sin \left( \psi + \delta_{\rm F} \right) + F_{y,{\rm F}} \cos \left( \psi + \delta_{\rm F} \right) + F_{x,{\rm R}} \sin \left( \psi + \delta_{\rm R} \right) + F_{y,{\rm R}} \cos \left( \psi + \delta_{\rm R} \right) \\
%     \nonumber I_{\rm T} \ddot{\psi} &=& F_{x,{\rm F}} a \sin(\delta_{\rm F}) + F_{y,{\rm F}} a \cos(\delta_{\rm F}) - F_{x,{\rm R}} b \sin(\delta_{\rm R}) - F_{y,{\rm R}} b \cos(\delta_{\rm R}),
%     \end{eqnarray}
% </p>
% <p>As discussed above, it may be more convenient to use the states \(v_{\rm T}\) e \(\alpha_{\rm T}\) instead of \(\dot{x}\) e \(\dot{y}\). The respective time derivatives are</p>
% <p>
%     \begin{eqnarray}
%     \nonumber \ddot{x} &=& \dot{v}_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) - v_{\rm T} \left( \dot{\psi} + \dot{\alpha}_{\rm T} \right) \sin \left( \psi + \alpha_{\rm T} \right) \\
%     \ddot{y} &=& \dot{v}_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) + v_{\rm T} \left( \dot{\psi} + \dot{\alpha}_{\rm T} \right) \cos \left( \psi + \alpha_{\rm T} \right). \label{eq:posvelanglesubs}
% \end{eqnarray}</p>
% <p>Substituting equation \ref{eq:posvelanglesubs} in \ref{eq:equationofmotion} we have</p>
% <p>
%     \begin{eqnarray}
%     \nonumber m_{\rm T} \left( \dot{v}_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) - v_{\rm T} \left( \dot{\psi} + \dot{\alpha}_{\rm T} \right) \sin \left( \psi + \alpha_{\rm T} \right) \right) &=& F_{x,{\rm F}} \cos \left( \psi + \delta_{\rm F} \right) - F_{y,{\rm F}} \sin \left( \psi + \delta_{\rm F} \right) + F_{x,{\rm R}} \cos \left( \psi + \delta_{\rm R} \right) - F_{y,{\rm R}} \sin \left( \psi + \delta_{\rm R} \right) \\
%     \nonumber m_{\rm T} \left( \dot{v}_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) + v_{\rm T} \left( \dot{\psi} + \dot{\alpha}_{\rm T} \right) \cos \left( \psi + \alpha_{\rm T} \right) \right) &=& F_{x,{\rm F}} \sin \left( \psi + \delta_{\rm F} \right) + F_{y,{\rm F}} \cos \left( \psi + \delta_{\rm F} \right) + F_{x,{\rm R}} \sin \left( \psi + \delta_{\rm R} \right) + F_{y,{\rm R}} \cos \left( \psi + \delta_{\rm R} \right) \\
%     \nonumber I_{\rm T} \ddot{\psi} &=& F_{x,{\rm F}} a \sin(\delta_{\rm F}) + F_{y,{\rm F}} a \cos(\delta_{\rm F}) - F_{x,{\rm R}} b \sin(\delta_{\rm R}) - F_{y,{\rm R}} b \cos(\delta_{\rm R}),
%     \end{eqnarray}
% </p>
% <p>Manipulating and simplifying</p>
% <p>
%     \begin{eqnarray}
%     \nonumber \dot{v}_{\rm T} &=& \frac{F_{x,{\rm F}} \cos \left( \alpha_{\rm T} - \delta_{\rm F} \right) + F_{x,{\rm R}} \cos \left( \alpha_{\rm T} -\delta_{\rm R} \right) + F_{y,{\rm F}} \sin \left( \alpha_{\rm T} - \delta_{\rm F} \right) + F_{y,{\rm R}} \sin \left( \alpha_{\rm T} - delta_{\rm R} \right) }{m_{\rm T}} \\
%     \nonumber \dot{\alpha}_{\rm T} &=&  \frac{- F_{x,{\rm F}} \sin \left( \alpha_{\rm T} - \delta_{\rm F} \right) - F_{x,{\rm R}} \sin \left( \alpha_{\rm T} - \delta_{\rm R} \right) + F_{y,{\rm F}} \cos \left( \alpha_{\rm T} - \delta_{\rm F} \right) + F_{y,{\rm R}} \cos \left( \alpha_{\rm T} - \delta_{\rm R} \right) - m_{\rm T} v \dot{\psi}}{m_{\rm T} v_{\rm T}} \\
%     \nonumber \ddot{\psi} &=& \frac{F_{x,{\rm F}} a \sin \delta  + F_{y,{\rm F}} a \cos \delta - F_{y,{\rm R}} b}{I_{\rm T}}.
%     \end{eqnarray}
% </p>
% <p>Therefore, the set of states variables is</p>
% <p>
%     \begin{eqnarray}
%     \nonumber {\rm x}_1 &=& x 				\\
%     \nonumber {\rm x}_2 &=& y 				\\
%     \nonumber {\rm x}_3 &=& \psi              \\
%     \nonumber {\rm x}_4 &=& v_{\rm T} 		\\
%     \nonumber {\rm x}_5 &=& \alpha_{\rm T} 	\\
%     \nonumber {\rm x}_6 &=& \dot{\psi}
%     \end{eqnarray}
% </p>
% <p>and the state equations are</p>
% <p>
%     \begin{eqnarray}
%     \nonumber \dot{{\rm x}}_1 &=& v_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) \\
%     \nonumber \dot{{\rm x}}_2 &=& v_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) \\
%     \nonumber \dot{{\rm x}}_3 &=& \dot{\psi} \\
%     \nonumber \dot{{\rm x}}_4 &=& \frac{F_{x,{\rm F}} \cos \left( \alpha_{\rm T} - \delta_{\rm F} \right) + F_{x,{\rm R}} \cos \left( \alpha_{\rm T} -\delta_{\rm R} \right) + F_{y,{\rm F}} \sin \left( \alpha_{\rm T} - \delta_{\rm F} \right) + F_{y,{\rm R}} \sin \left( \alpha_{\rm T} - delta_{\rm R} \right) }{m_{\rm T}} \\
%     \nonumber \dot{{\rm x}}_5 &=&  \frac{- F_{x,{\rm F}} \sin \left( \alpha_{\rm T} - \delta_{\rm F} \right) - F_{x,{\rm R}} \sin \left( \alpha_{\rm T} - \delta_{\rm R} \right) + F_{y,{\rm F}} \cos \left( \alpha_{\rm T} - \delta_{\rm F} \right) + F_{y,{\rm R}} \cos \left( \alpha_{\rm T} - \delta_{\rm R} \right) - m_{\rm T} v \dot{\psi}}{m_{\rm T} v_{\rm T}} \\
%     \dot{{\rm x}}_6 &=& \frac{F_{x,{\rm F}} a \sin \delta  + F_{y,{\rm F}} a \cos \delta - F_{y,{\rm R}} b}{I_{\rm T}}. \label{eq:stateequationnonlinear3dof}
%     \end{eqnarray}
% </p>
% <p>The nonlinear model of the package uses equations in \ref{eq:stateequationnonlinear3dof} with slip angles defined in \ref{eq:slipanglesnewvariables}</p>
% </html>

EQ = ddTddq-dTdq;
EQ = subs(EQ,symfunVariables,symVariables);
EQ = formula(simplify(subs(EQ,[dx dy ddx ddy],[dx_Alt dy_Alt ddx_Alt ddy_Alt])));

dvar = [dVT dALPHAT ddPSI];

solution = solve(EQ-Q,dvar);

dq_Alt = formula(subs(dq,[dx dy ddx ddy],[dx_Alt dy_Alt ddx_Alt ddy_Alt]));

f1 = dq_Alt.';
f2 = [ simplify(solution.dVT) ; simplify(solution.dALPHAT) ; simplify(solution.ddPSI)];
f = [ f1 ; f2];
disp(f)

%% See Also
%
% <../../../index.html Home>
%
