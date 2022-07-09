function [x, y, vx, vy, R, agent_u] = init_imazu( imazu_counter, Radius, deltaT, timesteps)
agent_u = 1+rand();
delta_timesteps = round(Radius/(agent_u * deltaT)); % additional timesteps to compensate Radius
delta_x = agent_u * (timesteps + delta_timesteps)*deltaT;
switch imazu_counter
    case 1  
        R = delta_x * 0.7 * rand();
        u = 2*rand();
        theta = deg2rad(0);
        [x, y, vx, vy] = state_polar2cart(theta, u, R);
        
    case 2
        R = delta_x * 0.75 * rand();        
        u = 2*rand();
        theta = deg2rad(-90);
        [x, y, vx, vy] = state_polar2cart(theta, u, R);
        
    case 3
        R = delta_x * 0.75 * rand();
        u = 2*rand();
        theta = deg2rad(0);
        [x, y, vx, vy] = state_polar2cart(theta, u, R);
        x = rand()*x;        
        
    case 4
        R = delta_x * 0.75 * rand();
        u = 2*rand();
        theta = deg2rad(135);
        [x, y, vx, vy] = state_polar2cart(theta, u, R);        
        
    case 5
        R = delta_x * 1.5 * rand();
        u = 2*rand(1,2);
        theta = [deg2rad(0) deg2rad(-90)] ;
        [x, y, vx, vy] = state_polar2cart(theta, u, R); 
        
    case 6
        R = delta_x * [rand() rand()];
        u = 2* rand(1,2);
        theta = [deg2rad(-170) deg2rad(-135)] ;
        [x, y, vx, vy] = state_polar2cart(theta, u, R);
        R = R(2);        
        
    case 7
        R = delta_x *0.9* [rand() rand()];
        u = 2* rand(1,2);
        theta = [deg2rad(-180) deg2rad(-135)] ;
        [x, y, vx, vy] = state_polar2cart(theta, u, R);
        R = R(2);    

    case 8
        R = delta_x * 1.5 * rand();
        u = 2* rand(1,2);
        theta = [deg2rad(0) deg2rad(-90)] ;
        [x, y, vx, vy] = state_polar2cart(theta, u, R);         
                
    case 9
        R = delta_x *0.75* rand();
        u = 2*rand(1,2);
        theta = [deg2rad(-150) deg2rad(-90)] ;
        [x, y, vx, vy] = state_polar2cart(theta, u, R);
                        
    case 10
        R = delta_x *0.75* [rand() rand()];
        u = 2*rand(1,2);
        theta = [deg2rad(165) deg2rad(-90)] ;
        [x, y, vx, vy] = state_polar2cart(theta, u, R);
        R = R(2);    
        
    case 11
        R = delta_x *0.75* rand();
        u = 2* rand(1,2);
        theta = [deg2rad(-150) deg2rad(90)] ;
        [x, y, vx, vy] = state_polar2cart(theta, u, R);  
                        
    case 12
        R = delta_x *0.75* [rand() rand()];
        u = 2*rand(1,2);
        theta = [deg2rad(-170) deg2rad(-135)] ;
        [x, y, vx, vy] = state_polar2cart(theta, u, R);  
        R = R(2);

    case 13
        R = delta_x *1* rand();
        R(2) = R(1) + 1 * rand() * delta_x;
        R(3) = R(1);
        u = 2* rand(1,3);
        theta = deg2rad([135 170 0]) ;
        [x, y, vx, vy] = state_polar2cart(theta, u, R);    
        R = R(3);
        
    case 14
        R = delta_x *0.75* rand();
        R(2) = R(1) + 0.25 * rand() * delta_x;
        R(3) = R(1);
        u = 2* rand(1,3);
        theta = deg2rad([-135 -170 -90]) ;
        [x, y, vx, vy] = state_polar2cart(theta, u, R);    
        R = R(3);        
        
    case 15
        R = delta_x *0.75* rand();
        R(2) = R(1) + 0.75 * rand() * delta_x;
        R(3) = R(2);
        u = 2* rand(1,3);
        theta = deg2rad([-180 -135 -90]) ;
        [x, y, vx, vy] = state_polar2cart(theta, u, R);    
        R = R(3);

    case 16
        R = delta_x *0.75* rand();
        u = 2* rand(1,3);
        theta = deg2rad([90 135 -90]) ;
        [x, y, vx, vy] = state_polar2cart(theta, u, R);         
        
    case 17
        R = delta_x *0.75* rand();
        R(2) = delta_x *0.75* rand();
        R(3) = R(2) + delta_x *0.5* rand();
        u = 2* rand(1,3);
        theta = deg2rad([-180 170 -135]) ;
        [x, y, vx, vy] = state_polar2cart(theta, u, R);    
        R = R(3);        
        
    case 18
        R = delta_x *0.75* rand();
        R(2) = R(1) + delta_x *0.25* rand();
        R(3) = R(2);        
        u = 2* rand(1,3);
        theta = deg2rad([-45 -165 -150]) ;
        [x, y, vx, vy] = state_polar2cart(theta, u, R);    
        R = R(1);         
        
    case 19
        R = delta_x *1* rand();
        R(2) = R(1) + delta_x *0.25* rand();
        R(3) = R(2);        
        u = 2* rand(1,3);
        theta = deg2rad([-45 -165 165]) ;
        [x, y, vx, vy] = state_polar2cart(theta, u, R);    
        R = R(1);         

    case 20
        R = delta_x *0.5* rand();
        R(2) = R(1) + delta_x *0.5* rand();
        R(3) = R(2);        
        u = 2* rand(1,3);
        theta = deg2rad([180 -165 -90]) ;
        [x, y, vx, vy] = state_polar2cart(theta, u, R);    
        R = R(3);         

     case 21
        R = delta_x *1* rand();
        R(2) = R(1) + delta_x *0.25* rand();
        R(3) = R(2);        
        u = 2* rand(1,3);
        theta = deg2rad([-90 -165 165]) ;
        [x, y, vx, vy] = state_polar2cart(theta, u, R);    
        R = R(1);     
        
    case 22
        R = delta_x *0.5* rand();
        R(2) = R(1) + delta_x *0.5* rand();
        R(3) = R(2);        
        u = 2* rand(1,3);
        theta = deg2rad([180 -135 -90]) ;
        [x, y, vx, vy] = state_polar2cart(theta, u, R);    
        R = R(3);         
        

        
        
end



end






