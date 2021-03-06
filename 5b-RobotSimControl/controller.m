
function u = controller(params, t, phi, phidot)
  % STUDENT FILLS THIS OUT
  % 
  % Initialize any added state like this:
  % 
  
   persistent newstate time
   if isempty(newstate)
     % initialize
     newstate =0 ;
    
   end
   if isempty(time)
       time=0;
   end
  % 
  dt=t-time;
  error=(0-phi);
  newstate=newstate+error*dt;
  kp=20;
  kd=2;
  ki=500;
  u = 0;
  u=-(kp*error+ki*newstate+kd*(0-phidot));
  time=t;
  
end

