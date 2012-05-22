classdef PlanePlant < SmoothRobotLibSystem
% Defines the dynamics for the powered plane
  
  properties
    m = 0.49;
    g = 9.81; % m/s^2
    Cmbreaks = [-120 -14.8 -4.5 8.4 19 120];
    Cmcoeffs =[      0    0.0746
               -0.0084    0.0733
                0.0012   -0.0134
               -0.0061    0.0024
                     0   -0.0653];
                 
    Cdcoeffs = [-0.0086    1.6993
                -0.0333    1.3982
                -0.0160    0.4169
                 0.0013    0.0081
                 0.0217    0.0205
                 0.0338    0.4200
                 0.0100    1.5000];
    Cdbreaks = [-90 -55 -28 -2.45 6.9 28 60 90];
    
    Clbreaks = [-90 -40 -20 -15 -6.5 8.9 11.85 45 110];
    Clcoeffs =[-0.0213    0.0643
                0.0190   -1.0000
               -0.0280   -0.6140
                0.0184   -0.7600
                0.1069   -0.6149
               -0.1420    1.0262
                0.0143    0.5905
               -0.0210    1.0500];
    Cmpp;
    Clpp;
    Cdpp;
  end
  
  methods
    function obj = PlanePlant()
      obj = obj@SmoothRobotLibSystem(12,0,5,12,0);
%     obj = setSimulinkParam(obj,'InitialStep','1e-3','MaxStep','0.05');
    obj.Cmpp = mkpp(obj.Cmbreaks, obj.Cmcoeffs);   
    obj.Cdpp = mkpp(obj.Cdbreaks,obj.Cdcoeffs);
    obj.Clpp = mkpp(obj.Clbreaks,obj.Clcoeffs);
    end
    
    function xdot = dynamics(obj,t,x,u)

        
        % Dynamics of the pitch plant system based on the prop-hang data
        % @param t time
        % @param x state: x = 
        %  Plane's X coordinate faces forward, Y to the right, and Z down.
        %  x(1):Pn    (North or forward Position, Earth frame)
        %  x(2):Pe    (East or y position, Earth frame)
        %  x(3):Pd     (z position, Earth frame)
        %  x(4):phi   (roll Euler angle)
        %  x(5):theta (pitch Euler angle)
        %  x(6):psi   (yaw Euler angle)
        %  x(7):U     (X-velocity, body frame)
        %  x(8):V     (Y-velocity, body frame)
        %  x(9):W     (Z-velocity, body frame)
        %  x(10):P    (Angular Rate Vector of roll, body frame)
        %  x(11):Q    (Angular Rate Vector of pitch, body frame)
        %  x(12):R    (Angular Rate Vector of yaw, body frame)
        %  
        %  u(1):thr   (Thrust command)
        %  u(2):elev  (Elevator Command)
        %  u(3):rud   (Rudder Command)
        %  u(4):ail   (Aileron Command)
        % Thrust numbers are approximate.  elev, rud, ail are [deg]
        %{ To do:
        % Make sure sign of aileron input corresponds to the appropriate deflection
        %}
        
        %x(3) = -x(3);
        Pn = x(1);
        Pe = x(2);
        Pd = x(3);
        phi = x(4);
        theta = x(5);
        psi = x(6);
        U = x(7);
        V = x(8);
        W = x(9);
        P = x(10);
        Q = x(11);
        R = x(12);
        
        %Throttle signal is 0-255
        thr = u(1);
%         if thr<20
%             thr = 0;
%         end
        %positive elevator is deflection up (pitches up)
        elev = u(2);% input in degrees of deflection--check sign
        %positive rudder is deflection to the right
        rud = u(3);% input in degrees of deflection-check sign
        ailL = u(4); % input in degrees of deflection
        ailR = u(5); % input in degrees of deflection
        
        % This model uses the basic equations from Stevens' Aircraft
        % Control and Simulation TL678.S74, with the following
        % modifications:
        % There is a collective added to angle of attack, so when the
        % airplane is at level flight, the AoA is a few degrees.
        % The wings are broken down into four sections, each which produces
        % its own lifting (torque) force on the airplane.  These sections
        % are the left and right inner and outer wings.  The inner sections
        % experience a prop-wash velocity, while the outer wings only
        % experience a free-stream (airspeed) velocity.
        %added a viscous damping term for rotational velocity
        % can/should add in terms for viscous damping for things like roll?
        
        % Estimates include:
        % Drag on the rest of the aircraft
        % Throttle/thrust relationship
        % Wing collective AoA
        % Prop Wash velocities
        
        %Known Parameters (MKS units):
        wing_area_out = .204*.227;%m^2
        wing_area_in = .204*.05; % m^2
        wing_total_area = 2*wing_area_out + 2*wing_area_in;
        out_dist = .243; %Moment arm of outer wing section
        in_dist = .055; %Moment arm of inner wing section
        elev_area = .0217;
        elev_arm = .49;
        rudder_area = .00850; 
        rudder_arm = .49; % m
        stab_area = .0092; % m^2
        stab_arm = .4; %m 
        obj.m = 0.49; % kg
        obj.g = 9.81; % m/s^2
        %Moments of Inertia in kg/m^2 (cross terms 100x smaller)
        Jx = .00515;
        Jy = .00881;
        Jz = .00390;
                
        %True Airspeed of aircraft, Vt
        vel = sqrt(U^2 + V^2 + W^2); 
  
        %Estimated parameters:
        %Wing collective AoA in degrees. Set this on airplane
        wing_collective = 0; 
        %20 is arbitrary.  Need a better mapping (from data) of throttle 
        %setting from controller (0-255) to the prop wash airstream velocity
        tfac1 = 40;
        wash = U + (thr)/tfac1; % Prop wash velocity in m/s
        
        %Angle of Attack (degrees):
        alpha = atan2(W,U)*180/3.1416 + wing_collective;
        %alpha = atan(W/U)*180/3.1416 + wing_collective;
        
        %Sideslip angle
        beta = atan2(V,sqrt(vel^2-V^2))*180/3.1416;
        %beta = asin(V/vel)*180/3.1416;
          
         
        %Lift force generated by wing components.
        %Should I use Vt or just x-velocity?
        %Cl expects units of degrees
        %lift = dynamic pressure * area * Coefficient of lift.
        left_out_lift = pressure(U) * wing_area_out * Cl(obj,alpha-ailL); %,cl1,cl2,cl3,cl4);
        left_in_lift = pressure(wash) * wing_area_in * Cl(obj,alpha-ailL); %,cl1,cl2,cl3,cl4);
        right_out_lift = pressure(U) * wing_area_out * Cl(obj,alpha+ailR); %,cl1,cl2,cl3,cl4);
        right_in_lift = pressure(wash) * wing_area_in * Cl(obj,alpha+ailR); %,cl1,cl2,cl3,cl4);
        
        %includes lift terms from flat plate theory of elevator
        %.11 * angle of attack in degrees = Coefficient of lift for a flat plate
        % see below for link on flat plate theory
        lift = left_out_lift + left_in_lift + right_out_lift + right_in_lift;
        elev_lift = pressure(wash) * elev_area * ... likely a small term
            0.11*(alpha-wing_collective-elev); %angle of deflection of Elevator
        lift = lift + elev_lift;
        
        %Lift force generated by wing components.
        %Should I use Vt or just x-velocity?
        left_out_drag = pressure(U) * wing_area_out * Cd(obj,alpha-ailL);
        left_in_drag = pressure(wash) * wing_area_in * Cd(obj,alpha-ailL);
        right_out_drag = pressure(U) * wing_area_out * Cd(obj,alpha+ailR);
        right_in_drag = pressure(wash) * wing_area_in * Cd(obj,alpha+ailR);
        
        
        % ESTIMATED( * frontal area of elev)
        %1.2 is an approx of the Coefficient of Drag for a flat plate
        %perpendicular to the flow.  The elevator, for drag purposes, is
        %modeled as a flat plate whose area is equal to the projected frontal area of
        %the elevator (area * sin(deflection))
        elev_drag = pressure((wash+U)/2) * 1.2 * elev_area * sin((alpha-wing_collective-elev)*3.1415/180);
        wing_drag = left_out_drag + left_in_drag + right_out_drag + right_in_drag;
        %The body drag has no strict linkage to the actual airplane.  I
        %pulled the numbers from my head as what seemed reasonable.
        %Excellent candidates for fits from experimental data. The body
        %drag term takes all the non-wing frontal areas of the plane, and
        %treats them as a flat plate perpendicular to the airspeed, and
        %calculates a drag for that thin flat plate.
        %estimated frontal area of cameras + body. 1.2~cd for flat plate
        body_drag = 1.2 * pressure(wash) * .004;
        drag = wing_drag + elev_drag + body_drag;
        
        drag_fac = 2.02;
        body_z_drag = drag_fac*1.2 * pressure(W) * wing_total_area;
        
        %Estimated linearly from ~180=4.5N of force from prop hang.
        %i.e. a throttle setting of 180 out of 255 holds the 4.5Newton
        %plane in prop hang.  But another great opportunity for a fitted
        %parameter/relationship from data.
        tfac2 = 180;
        thrust = 4.5*thr/tfac2;
        
        %Roll torque neglects the rolling torque generated by the rudder
        roll_torque = (left_out_lift*out_dist + left_in_lift*in_dist)...
            - (right_in_lift*in_dist + right_out_lift*out_dist);
        roll_torque_fac = 1.47;
        roll_torque = roll_torque_fac*roll_torque;
        
        wing_total_area = (wing_area_out + wing_area_in);
        %(Cm*dynamic pressure*wing reference area*average chord)=pitching moment
        % .204 is the average chord of the wing
        pitch_torque = -elev_lift*elev_arm + ...
            (pressure(U) * Cm(obj,alpha) * wing_total_area * .204);
        pitch_torque_fac = 1;
        pitch_torque = pitch_torque_fac*pitch_torque;
        
        %Stabilizing force from aircraft yawing
        stab_force = -sign(R)*pressure(R*stab_arm) * (stab_area+rudder_area);
        %include rudder area in the V term of the stabilizing force?  The
        %rudder isn't always perpendicular to V, but it is close most of
        %the time.
        %stabilizing force from sideslip
        stab_force = stab_force + (-sign(V))*pressure(V)*stab_area*1.2;
        %Coefficient of lift for a flat plate is .11*(angle of attack in
        %degrees) http://www.onemetre.net/design/downwash/Circul/Circul.htm
        rudder_force = -pressure(wash) * rudder_area * .11 * (beta + rud);
        
        yaw_torque = stab_force * stab_arm + rudder_force * -rudder_arm;
        
        Pn_dot = U * cos(theta)*cos(psi) + ...
            V*(-cos(phi)*sin(psi) + sin(phi)*sin(theta)*cos(psi)) +...
            W*(sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi));
        Pe_dot = U * cos(theta)*sin(psi) + ...
            V*(cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi)) +...
            W*(-sin(phi)*cos(psi) + cos(phi)*sin(theta)*sin(psi));
        h_dot = U*sin(theta) - V*sin(phi)*cos(theta) - W*cos(phi)*cos(theta);
        phi_dot = P + tan(theta)*(Q*sin(phi) + R*cos(phi));
        theta_dot = Q*cos(phi) - R*sin(phi);
        psi_dot = (Q*sin(phi) + R*cos(phi))/cos(theta);
        U_dot = R*V - Q*W - obj.g * sin(theta) + (thrust - drag)/obj.m; % done
        V_dot = -R*U + P*W + obj.g * sin(phi)*cos(theta) + (stab_force + rudder_force)/obj.m;
        W_dot = Q*U - P*V + obj.g * cos(phi) * cos(theta) - lift/obj.m - sign(W)*body_z_drag/obj.m; 
        P_dot = (Jy-Jz)*R*Q/Jx + roll_torque/Jx;
        Q_dot = (Jz-Jx)*P*R/Jy + pitch_torque/Jy;
        R_dot =  yaw_torque/Jz+(Jx-Jy)*P*Q/Jz;
        
        xdot = [Pn_dot Pe_dot -h_dot phi_dot theta_dot psi_dot U_dot V_dot...
            W_dot P_dot Q_dot R_dot];
    
    
    function cl = Cl(obj,a) %polars from xfoil, then with fit lines/curves
        %cl = -5.3265*(10^-10) * a^8 + 5.124*(10^-9) * a^7 + ...
        %    2.8475*(10^-7) * a^6 - 4.953*(10^-7) * a^5 + -5.5107*(10^-05) * a^4 ...
        %    - 5.0420*(10^-04) * a^3 + 0.003262 * a^2 + .1264 * a + .05551; 
        
        if a > 90
            a = 90;
        elseif a<-90
            a = -90;
        end
        cl = ppval(obj.Clpp,a); %ppval(obj.Clpp,a);
 
    end
    
    function cd = Cd(obj,a) % from xfoil, then fit with a curve
        %cd = .003706 -.003379*a + .001203*a^2 + .000009858 * a^3 - .000001767 * a^4;
        if a > 90
            a = 90;
        elseif a<-90
            a = -90;
        end
        cd = ppval(obj.Cdpp, a);
    end
    
     function cm = Cm(obj,a)
         %cm = -.009964+.001142*a +.0003032*a^2 - 4.353*(10^-5)*a^3 - 8.949*(10^-7)*a^4 +9.38*(10^-8)*a^5;
        if a > 90
            a = 90;
        elseif a<-90
            a = -90;
        end
         cm = ppval(obj.Cmpp, a);
    end
    
    function pre = pressure(vel) %Dynamic Pressure = .5 rho * v^2
        pre = .5 * 1.22148 * vel^2; % N/m^2
    end
    end
    
    function y = output(obj,t,x,u)
      y = x;
    end
    
    function x = getInitialState(obj)
      x = zeros(12,1);
    end
    
  end
  
end