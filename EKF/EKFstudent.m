
function xhat = EKFstudent(t, z)
  % In this exercise, you will batch-process this data: you are provided a vector of timestamps (of length T), and a 3xT matrix of observations, z.
  xhat = zeros(2,length(t));
   P =eye(2);
  Q = diag([100, 1]);
  %R = diag([0.0001, 0.01, 30]);
  R = diag([0.0020, 0.01, 75]);
  xhat(1,1)=atan2d(z(1,1),z(2,1));
  %tuning_q=1;
  %Del_t=t-[0;t(1:end-1,1)];
  xhat(2,1)=z(3,1);
  for i=2:length(t)
      %del_t=Del_t(i);
      del_t=t(i)-t(i-1);
       A=[1 del_t;...
          0 1];
      %Q=tuning_q*[del_t^3/3 del_t^2/2;...
      %          del_t^2/2   del_t];
     %prediction step
      xhat(:,i)=A*xhat(:,i-1);
      H = [cosd(xhat(1,i-1)) 0; -sind(xhat(1,i-1)) 0 ; 0 1]*0.50;%calculationg jacobian of funx H(theta(k-1)) w.r.t theta(k-1)
      h = [sind(xhat(1,i)); cosd(xhat(1,i)); xhat(2,i)];
       P=A*P*A'+Q;  %noise covariance matrix
       K=P* H'*pinv(H*P*H'+R); % calculate gain
     % update step
      xhat(:,i)=xhat(:,i)+K*(z(:,i)-h);
      P=P-K*H*P;
  end
  % Student completes this
end
