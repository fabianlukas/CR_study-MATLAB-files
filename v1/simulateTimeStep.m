function [x,y,theta,u,v,r] = simulateTimeStep(x,y,theta,u,v,r,A_r,A_u)

            % constants
            deltaT = 0.1;
            J = 30;
            m = 15;
            
            % simualate timestep
            r_dot = 1/J * A_r;
            u_dot = 1/m * A_u;
            r = r + r_dot * deltaT;
            u = u + u_dot * deltaT;

            % rotation matrix
            R = [cos(theta) -sin(theta) 0;
                 sin(theta)  cos(theta) 0;
                 0              0       1];

            eta_dot = mtimes(R,[u;v;r]);
            eta = [x;y;theta];
            eta = eta + eta_dot * deltaT;

            x = eta(1);
            y = eta(2);
            theta = eta(3);

end

