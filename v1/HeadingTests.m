close all
phi = 3;
x = 0;
y = 0;


x_obst = -4;
y_obst = -1.3;
phi_obst = 1;

vx_obst = -1;
vy_obst = -1;




plot([x , x+(cos(phi))],[y, y+(sin(phi))])
hold on
plot(x,y,'*')

plot([x_obst , x_obst + vx_obst],[y_obst, y_obst+vy_obst])
plot(x_obst,y_obst,'*')
ylim([-5 5])
xlim([-5 5])
pbaspect([1 1 1])


theta = rad2deg(atan2(y_obst,x_obst));
phi =  rad2deg(phi);
theta = theta-phi

phi_obst = rad2deg(atan2(-vy_obst,-vx_obst));

theta2 = phi_obst-phi

 