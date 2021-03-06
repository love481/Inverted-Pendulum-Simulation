
function u = controllerNoisyEnc(params, t, obs, th, dth)
  % This is the starter file for the week5 assignment
  % Now you only receive noisy measurements for theta, and must use your EKF from week 3 to filter the data and get an estimate of the state
  % obs = [ay; az; gx] (same as last week)
  % New for 6b: you also have access to params.traj(t)

  % Template code (same as last week)
  xhat = EKFupdate(params, t, obs);
  phi = xhat(1);
  phidot = xhat(2);
   x=params.r*(th+phi);
  xdot=params.r*(dth+phidot);
  
  kp1=0.15;
  kd1=0.15;
  
  u1=kp1*(params.traj(t)-x)+kd1*(0-xdot);
  
  phides=asin(u1);
  
  kp2=0.1;
  kd2=0.02;
  
  u=-kp2*sin(phi-phides)-kd2*phidot;
end

function xhatOut = EKFupdate(params, t, z)
  % z = [ay; az; gx] with a* in units of g's, and gx in units of rad/s
  % You can borrow most of your week 3 solution, but this must only implement a single predict-update step of the EKF
  % Recall (from assignment 5b) that you can use persistent variables to create/update any additional state that you need.
 persistent ti PreState P
  if isempty(ti)
      ti=t;
      P =eye(2);
      PreState=[atan2(z(1),z(2));z(3)];
      xhatOut=PreState;
      return;
  end
  Q = diag([100, 1]);
  R = diag([0.0020, 0.01, 75]);
  del_t=t-ti;
  A=[1 del_t;...
          0 1];
   xhatOut=A*PreState;
   H = [cos(PreState(1)) 0; -sin(PreState(1)) 0 ; 0 1]*0.50;
    h = [sin(xhatOut(1)); cos(xhatOut(1)); xhatOut(2)];
    P=A*P*A'+Q;  %noise covariance matrix
       K=P* H'*pinv(H*P*H'+R); % calculate gain
     % update step
      xhatOut=xhatOut+K*(z(:,1)-h);
      P=P-K*H*P;
       ti=t;
       PreState=xhatOut;
  % Student completes this
end
