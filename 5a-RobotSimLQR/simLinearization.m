
params = struct();

params.g = 9.81;
params.mr = 0.25;
params.ir = 0.0001;
params.d = 0.1;
params.r = 0.02;

% 1. Learn how to use the symbolic toolbox in MATLAB
% you will need the state variables, and control input to be declared as symbolic
syms th phi dth dphi u
ddthphi=eom(params, th, phi, dth, dphi, u);
state=[th;phi;dth;dphi];
dd=[dth;dphi;ddthphi(1);ddthphi(2)];
 As= jacobian(dd,state);
 Bu= jacobian(dd,u);
 A=double(subs(As,[th, phi, dth, dphi ,u],[0,0,0,0,0]));
 B=double(subs(Bu,[th, phi, dth, dphi ,u],[0,0,0,0,0]));
% 2. call your "eom" function to get \ddot{q} symbolically
ctr=ctrb(A,B);
unco=length(A)-rank(ctr);
% 3. Linearize the system at 0 (as shown in lecture)
% You should end up with A (4x4), and b (4x1)
K=lqr(A,B,eye(4),1,0);
% 4. Check that (A,b) is  controllable
% Number of uncontrollable states should return 0

% 5. Use LQR to get K as shown in the lecture
