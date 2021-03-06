function qdd = eom(params, th, phi, dth, dphi, u)
  % This is the starter file for the week5 assignment

  % Provided params are
  % params.g: gravitational constant
  % params.mr: mass of the "rod"
  % params.ir: rotational inertia of the rod
  % params.d: distance of rod CoM from the wheel axis
  % params.r: wheel radius

  % Provided states are:
  % th: wheel angle (relative to body)
  % phi: body pitch
  % dth, dphi: time-derivatives of above
  % u: torque applied at the wheel 
  g=params.g ;
   m=params.mr;
   i=params.ir;
   l=params.d ;
   r=params.r;
   A=[m*r^2 ((m*r^2)+(m*r*l*cos(phi)));...
       (m*r^2)+(m*r*l*cos(phi)) ((m*r^2)+(m*l^2)+(2*m*r*l*cos(phi))+i)];
   B=[-u+m*r*l*dphi^2*sin(phi);...
        (m*(2*l*sin(phi)*dphi*(r*(dphi + dth) + l*cos(phi)*dphi) - 4*l^2*cos(phi)*sin(phi)*dphi^2 + l*sin(phi)*dphi^2*(2*r + 2*l*cos(phi))))/2 + (m*(2*l^2*cos(phi)*sin(phi)*dphi^2 - 2*l*sin(phi)*dphi*(r*(dphi + dth) + l*cos(phi)*dphi)))/2 + g*l*m*sin(phi)];
  % B=[-u+m*r*l*dphi^2*sin(phi);...
    
   %-(m*r*l*dphi^2*sin(phi))-(m*r*l*dphi^2*sin(phi)*dth)+(m*g*l*sin(phi))-(2*m*r*l*dphi^2*sin(phi))+(m*r*l*dth*dphi*sin(phi))];
  qdd = [0;0];
  qdd=linsolve(A,B);
  % THE STUDENT WILL FILL THIS OUT
  
end