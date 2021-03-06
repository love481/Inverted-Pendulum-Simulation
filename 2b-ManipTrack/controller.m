
function u = controller(params, t, X)
  u=[0; 0];
  % 1. write out the forward kinematics, such that p = FK(theta1, theta2)
  % 2. Let e = p - params.traj(t) be the task-space error
  % 3. Calculate the manipulator Jacobian J = d p / d theta
  % 4. Use a "natural motion" PD controller, u = - kp * J^T * e - kd * [dth1; dth2]
  th1 = X(1);
  th2 = X(2);
  dth1 = X(3);
  dth2 = X(4);
  kp=250;
  kd=2.3;
   p1 = params.l*[cos(th1);sin(th1)];
  p = p1 + params.l*[cos(th1+th2);sin(th1+th2)];
  e = p - params.traj(t);
  Jacobian=[-sin(th1)-sin(th1+th2) -sin(th1+th2);...
            cos(th1)+cos(th1+th2)  cos(th1+th2)];
    u = - kp * Jacobian' * e - kd * [dth1; dth2];    
        
end

