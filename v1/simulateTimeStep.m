function [x,y,theta,u,v,r] = simulateTimeStep(x,y,theta,u,v,r,A_r,A_u,DYNAMICS)
    deltaT = DYNAMICS.deltaT(DYNAMICS.SET);

    if DYNAMICS.SET == 1
        % constants
        J = 30;
        m = 15;

        % simualate timestep
        r_dot = 1/J * A_r;
        u_dot = 1/m * A_u;
        r = r + r_dot * deltaT;
        u = u + u_dot * deltaT;
        if u < 0
            u = 0;
        end

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
        
        
    elseif DYNAMICS.SET == 2

        m = 23.8;
        Iz = 1.76;
        xg = 0.046;
        Xudot = -2;
        Yvdot = -10;
        Yrdot = 0;
        Nvdot = 0;
        Nrdot  = -1;

        M = [m-Xudot,   0,    0;
            0,     m-Yvdot,    m*xg-Yrdot;
            0,     m*xg-Nvdot, Iz-Nrdot]; 
        M_inv = inv(M);



        xg = 0.046;

        Xu = -0.72253;
        Xuu = -1.32742;
        Xuuu = -5.86643;
        Yv = -0.88965;
        Yvv = -36.47287;

        Yrv = -0.805;
        Yvr = -0.845;
        Yrr = -3.450;
        Nrv = 0.13;
        Nr = -1.9;
        Nvv = 3.95645;
        Nv = 0.03130;
        Nrr = -0.750;
        Yr = -7.250;
        Nvr = 0.08;
        Xudot = -2;
        Yvdot = -10;
        Yrdot = 0;
        Nvdot = 0;

        % update nu vector    
        % g = np.array([0.0279*u*v^2 + 0.0342*v^2*r, 0.0912*u^2*v, 0.0156*u*r^2 + 0.0278*u*r*v^3])

        tau = [2+1.2*A_u , 0, A_r]';

        c13 = Yvdot*v+0.5*(Yrdot+Nvdot)*r-m*(xg*r+v);
        c23 = -Xudot*u+m*u;    
        C = [0, 0, c13; 0, 0, c23;-c13, -c23, 0];

        d11 = -Xu-Xuu*abs(u)-Xuuu*u^2;
        d22 = -Yv-Yvv*abs(v)-Yrv*abs(r);
        d23 = -Yr-Yvr*abs(v)-Yrr*abs(r);
        d32 = -Nv-Nvv*abs(v)-Nrv*abs(r);
        d33 = -Nr-Nvr*abs(v)-Nrr*abs(r);



        D = [d11, 0, 0; 0, d22, d23; 0, d32, d33];

        agent_nu = [u,v,r]';
        nu_dot = mtimes(M_inv, tau - mtimes(C+D, agent_nu));

        agent_nu = agent_nu + nu_dot * deltaT;


        % update eta vector
        R = [cos(theta), -sin(theta), 0;
            sin(theta), cos(theta),  0;
            0,           0,            1];
        agent_eta_dot = mtimes(R,agent_nu);
        agent_eta = [x;y;theta];
        agent_eta = agent_eta + agent_eta_dot * deltaT;
        
        x = agent_eta(1);
        y = agent_eta(2);
        theta = agent_eta(3);
        
        u = agent_nu(1);
        v = agent_nu(2);
        r = agent_nu(3);
        
    elseif DYNAMICS.SET == 3 % KVLCC2 Vessel
            C_b=          0.810;         % Block Coefficient
            Lpp=          320.0;         % Length over pependiculars (m)
            B=            58.;           % Overall width
            m=            312600*1020;  % Mass of ship as calculated by ▽*rho (displacement * water density)
            w_P0=         0.35;          % Assumed wake fraction coefficient
            J_int=        0.4;           % Intercept for the calculation of K_T (https://doi.org/10.1615/ICHMT.2012.ProcSevIntSympTurbHeatTransfPal.500)
            J_slo=       -0.5;           % Slope for the calculation of K_T
            x_G=          11.2;          % X-Coordinate of the center of gravity (m)
            x_P=         -160.0;         % X-Coordinate of the propeller (-0.5*Lpp)
            D_p=          9.86;          % Diameter of propeller (m)
            k_0=          0.2931;        % Same value as "J_int" | Propeller open water coefficients. 
            k_1=         -0.2753;
            k_2=         -0.1359;    
            C_1=          2.0;
            C_2_plus=     1.6;
            C_2_minus=    1.1;
            l_R=         -0.710;         % correction of flow straightening factor to yaw-rate
%             gamma_R=      None;      
            gamma_R_plus= 0.640;         % Flow straightening coefficient for positive rudder angles
            gamma_R_minus=0.395;         % Flow straightening coefficient for negative rudder angles
            eta_param=    0.626;         % Ratio of propeller diameter to rudder span
            kappa=        0.50;          % An experimental constant for expressing "u_R"
            A_R=          112.5;         % Moveable rudder area
            epsilon=      1.09;          % Ratio of wake fraction at propeller and rudder positions ((1 - w_R) / (1 - w_P))
            A_R_Ld_em=    1/46.8;        % Fraction of moveable Rudder area to length*draft
            f_alpha=      2.747;         % Rudder lift gradient coefficient (assumed rudder aspect ratio = 2)
            rho=          1020;          % Water density of seawater
            t_R=          0.387;         % Steering resistance deduction factor
            t_P=          0.220;         % Thrust deduction factor. TODO give this more than an arbitrary value
            x_H_dash=    -0.464;         % Longitudinal coordinate of acting point of the additional lateral force
            d=            20.8;          % Ship draft (Tiefgang)
            m_x_dash=     0.022;         % Non dimensionalized added masses coefficient in x direction
            m_y_dash=     0.223;         % Non dimensionalized added masses coefficient in y direction
            R_0_dash=     0.022;         % frictional resistance coefficient TODO Estimate this via Schoenherr's formula
            X_vv_dash=   -0.040;         % Hull derivatives
            X_vr_dash=    0.002;         % Hull derivatives
            X_rr_dash=    0.011;         % Hull derivatives
            X_vvvv_dash=  0.771;         % Hull derivatives
            Y_v_dash=    -0.315;         % Hull derivatives
            Y_r_dash=     0.083;         % Hull derivatives
            Y_vvv_dash=  -1.607;         % Hull derivatives
            Y_vvr_dash=   0.379;         % Hull derivatives
            Y_vrr_dash=  -0.391;         % Hull derivatives
            Y_rrr_dash=   0.008;         % Hull derivatives
            N_v_dash=    -0.137;         % Hull derivatives
            N_r_dash=    -0.049;         % Hull derivatives
            N_vvv_dash=  -0.030;         % Hull derivatives
            N_vvr_dash=  -0.294;         % Hull derivatives
            N_vrr_dash=   0.055;         % Hull derivatives
            N_rrr_dash=  -0.013;         % Hull derivatives
            I_zG=         2e12;          % Moment of inertia of ship around center of gravity (m*(0.25*Lpp)^2) (Point mass Inertia)
            J_z_dash=     0.011;         % Added moment of inertia coefficient
            a_H=          0.312;          % Rudder force increase factor
            
            rudder_angle_max = deg2rad(20);
            rud_angle = A_r * rudder_angle_max;
            
            nps = A_u;
            eta = [x;y;theta];
            vm = v;
            nu = [u;vm;r];
            nu_dot = [0;0;0];
            
            R = [cos(theta), -sin(theta), 0;
                sin(theta), cos(theta),  0;
                0,           0,            1];
            eta_dot_old = mtimes(R, nu);
            
            U = sqrt(u^2 + vm^2);

        if U == 0
            beta = 0.0;
            v_dash = 0.0;
            r_dash = 0.0;
        else
            beta = atan2(-vm, u) ;  % drift angle at midship position
            v_dash = vm / U     ;        % non-dimensionalized lateral velocity
            r_dash = r * Lpp / U;   % non-dimensionalized yaw rate
        end
        %---------------- hydrodynamic forces acting on ship hull ----------------
        X_H = (0.5 * rho * Lpp * d * (U^2) * ( ...
            - R_0_dash ...
            + X_vv_dash * (v_dash^2) ...
            + X_vr_dash * v_dash * r_dash ...
            + X_rr_dash * (r_dash^2) ...
            + X_vvvv_dash * (v_dash^4) ...
        ) ...
        );

        Y_H = (0.5 * rho * Lpp * d * (U^2) * ( ...
            Y_v_dash * v_dash ...
            + Y_r_dash * r_dash ...
            + Y_vvv_dash * (v_dash^3) ...
            + Y_vvr_dash * (v_dash^2) * r_dash ...
            + Y_vrr_dash * v_dash * (r_dash^2) ...
            + Y_rrr_dash * (r_dash^3) ...
        ) ...
        );

        N_H = (0.5 * rho * (Lpp^2) * d * (U^2) * ( ...
            N_v_dash * v_dash ...
            + N_r_dash * r_dash ...
            + N_vvv_dash * (v_dash^3) ...
            + N_vvr_dash * (v_dash^2) * r_dash ...
            + N_vrr_dash * v_dash * (r_dash^2) ...
            + N_rrr_dash * (r_dash^3) ...
        ) ...
        );

        %---------------- longitudinal surge force due to propeller ----------------
        % redefine
        beta_P = beta - (x_P/Lpp) * r_dash;
        if beta_P >= 0
            C_2 = C_2_plus;  
        else
            C_2 = C_2_minus;
        end

            tmp = 1-exp(-C_1*abs(beta_P))*(C_2-1);
            w_P = 1-(1-w_P0)*(1+tmp);

        if nps == 0.0  % no propeller movement, no advance ratio
            J = 0.0;
        else
            J = (1 - w_P) * u / (nps * D_p);  % propeller advance ratio
        end

            % propeller thrust open water characteristic
            K_T = k_0 + (k_1 * J) + (k_2 * J^2);


        X_P = (1 - t_P) * rho * K_T * nps^2 * D_p^4;


        %--------------------- hydrodynamic forces by steering ----------------------
        % effective inflow angle to rudder in maneuvering motions
        beta_R = beta - l_R * r_dash;

        % flow straightening coefficient

            if beta_R < 0.0
                gamma_R = gamma_R_minus;
            else
                gamma_R = gamma_R_plus;
            end

        % lateral inflow velocity components to rudder
        v_R = U * gamma_R * beta_R;

        % longitudinal inflow velocity components to rudder
        if J == 0.0
            u_R = 0.0;
        else
            u_R = u * (1 - w_P) * epsilon * sqrt( ...
                eta_param * (1.0 + kappa * (sqrt(1.0 + 8.0 * K_T / (pi * J^2)) - 1))^2 + (1 - eta_param) ...
            );
        end
        % rudder inflow velocity
        U_R = sqrt(u_R^2 + v_R^2);

        % rudder inflow angle
        alpha_R = rud_angle - atan2(v_R, u_R);

        % normal force on rudder
        F_N = 0.5 * A_R * rho * f_alpha * (U_R^2) * sin(alpha_R);

        % longitudinal surge force around midship by steering
        X_R = -(1 - t_R) * F_N * sin(rud_angle);

        % lateral surge force by steering
        Y_R = -(1 + a_H) * F_N * cos(rud_angle);

        % redimensionalize x_H
        x_H = x_H_dash * Lpp;

        % yaw moment around midship by steering
        N_R = -(-0.5 + a_H * x_H) * F_N * cos(rud_angle);


       
            [X_C, Y_C, N_C] = deal(0);


        %-------------------------- Equation solving ----------------------------
        % added masses and added moment of inertia
        m_x = m_x_dash * (0.5 * rho * (Lpp^2) * d);
        m_y = m_y_dash * (0.5 * rho * (Lpp^2) * d);
        J_z = J_z_dash * (0.5 * rho * (Lpp^4) * d);

        X = X_H + X_R + X_P + X_C;
        Y = Y_H + Y_R + Y_C;
        N_M = N_H + N_R + N_C;

        % longitudinal acceleration
        d_u = (X + (m + m_y) * vm * r + x_G * m * (r^2)) / (m + m_x);

        % lateral acceleration
        f = I_zG + J_z + (x_G^2) * m;

        d_vm_nom = Y - (m + m_x)*u*r - x_G * m * N_M/f + x_G^2 * m^2 * u * r/f;
        d_vm_den = m + m_y - (x_G^2 * m^2)/f;
        d_vm = d_vm_nom / d_vm_den;

        % yaw rate acceleration
        d_r = (N_M - x_G * m * (d_vm + u * r)) / f;

        nu_dot = [d_u; d_vm; d_r];
            
         nu = nu + nu_dot * deltaT;
         
            R = [cos(theta), -sin(theta), 0;
                sin(theta), cos(theta),  0;
                0,           0,            1];
        % find new eta_dot via rotation
        eta_dot_new = mtimes(R, nu);

        % trapezoidal update of positions
        eta = eta + 0.5 * (eta_dot_old + eta_dot_new) * deltaT;

        % transform heading to [0, 2pi)
        %eta[2] = angle_to_2pi(eta[2]) %????????????????????????????????????????????
        
        x = eta(1);
        y = eta(2);
        theta = eta(3);
        
        u = nu(1);
        v = nu(2);
        r = nu(3);
    end


end

