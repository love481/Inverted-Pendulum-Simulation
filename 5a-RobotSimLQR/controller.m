
function u = controller(params, t, X)
  % You have full state feedback available
  u=0;
  K=-[ 1.0000  113.1949    1.2465   13.9340];
  u=-K*X;
  % After doing the steps in simLinearization, you should be able to substitute the linear controller u = -K*x
  
end

