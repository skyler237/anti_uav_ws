tr = 1;

v_fleet = 12.9;
v_intr = 2.17;

d = 103;
theta = 0.05*pi/180;
phi = 0*pi/180;

t_fleet = d*(tan(theta))/v_fleet + tr;
t_intr = d*(sec(theta))/v_intr;
diff = t_intr - t_fleet;

t_fleet2 = (d*(sin(theta)))/(v_fleet*(cos(theta + phi))) + tr;
t_intr2 = (d*(cos(phi))/(v_intr*(cos(theta+phi))));
diff2 = t_intr2 - t_fleet2;

phi_d = 180/pi*(acos((d*sin(theta)/tr)*(cos(theta)/v_intr - sin(theta)/v_fleet)) - theta);

phi1 = -pi/2;
phi2 = pi/2;

epsilon = .01;
timeout = 5000;
while abs(diff2) > epsilon
   phi = (phi1 + phi2)/2;
   t_fleet2 = (d*(sin(theta)))/(v_fleet*(cos(theta + phi))) + tr;
   t_intr2 = (d*(cos(phi))/(v_intr*(cos(theta+phi))));   
   diff2 = t_intr2 - t_fleet2;
   if(diff2 < 0) 
       phi1 = phi;
   else
       phi2 = phi;
   end  
   timeout = timeout - 1;
   if(timeout == 0)
      sprintf('ERROR: phi computation timeout')
      break;
   end
end

phi_deg = phi*180/pi
t_fleet2 = (d*(sin(theta)))/(v_fleet*(cos(theta + phi))) + tr
t_intr2 = (d*(cos(phi))/(v_intr*(cos(theta+phi))))
diff2 = t_intr2 - t_fleet2
delta_d = v_fleet*sin(phi)