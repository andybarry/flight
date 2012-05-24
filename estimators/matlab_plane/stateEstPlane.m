% Plane plant
p = PlanePlant;

disp('Starting state estimation.');

% Initialize state variables
s_last = [];
u_last = zeros(5,1);
t_last = [];
s_new = zeros(12,1);

% lcm stuff
lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
aggregator.setMaxMessages(1);  % make it a last-message-only queue

lc.subscribe('wingeron_x',aggregator); 

storage = LCMStorage('wingeron_u');
zeros_storage = LCMStorage('wingeron_gains');



% Estimator loop
while true
    % Get x,y,z,roll,pitch,yaw and u
    msg = getNextMessage(aggregator);  % will block until receiving a message
    msg = lcmtypes.lcmt_optotrak(msg.data);
    
    if ~isempty(msg)
        x = msg.x(1)/1000; % Convert to m
        y = msg.y(1)/1000;
        z = msg.z(1)/1000;
        roll = msg.yaw(1); % Optotrak flips yaw and roll!!
        pitch = msg.pitch(1);
        yaw = msg.roll(1);
        t = msg.timestamp/1000; % Comes in ms
        
        % Get control
        umsg = storage.GetLatestMessage(100);
        zeros_msg = zeros_storage.GetLatestMessage(100);
        
        if ~isempty(umsg) && ~isempty(zeros_msg)
            umsg = lcmtypes.lcmt_wingeron_u(umsg.data);
            zeros_msg = lcmtypes.lcmt_wingeron_gains(zeros_msg.data);
            
            umsg = commandsToDegrees(zeros_msg,umsg);
            
            throttle = umsg.throttleFront;
            rudder = umsg.rudder;
            elevator = umsg.elevator;
            aileronL = umsg.aileronLeft;
            aileronR = umsg.aileronRight;
            
            
            u = [throttle;elevator;rudder;aileronL;aileronR];
                
        else
            u = u_last;
        end
        

        statemsg = lcmtypes.lcmt_optotrak_xhat();
        
        % Compute derivatives of positions, angles
        if (~isempty(s_last))

            if (abs(t-t_last) > eps)
                xdot = (x-s_last(1))/(t-t_last);
                ydot = (y-s_last(2))/(t-t_last);
                zdot = (z-s_last(3))/(t-t_last);
                rolldot = (roll-s_last(4))/(t-t_last);
                pitchdot = (pitch-s_last(5))/(t-t_last);
                yawdot = (yaw-s_last(6))/(t-t_last);
            else
                xdot = 0;
                ydot = 0;
                zdot = 0;
                rolldot = 0;
                pitchdot = 0;
                yawdot = 0;
            end

            % Convert derivatives to U,V,W,P,Q,R
            Bb = [cos(pitch)*cos(yaw), cos(pitch)*sin(yaw), -sin(pitch); ...
                 -cos(roll)*sin(yaw)+sin(roll)*sin(pitch)*cos(yaw), cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw), sin(roll)*cos(pitch); ...
                 sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw), -sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw), cos(roll)*cos(pitch)];

            UVW = Bb*[xdot;ydot;zdot];
            U = UVW(1);
            V = UVW(2);
            W = UVW(3);

            Ew = [1, tan(pitch)*sin(roll), tan(pitch)*cos(roll); ...
                  0, cos(roll), -sin(roll); ...
                  0, sin(roll)/cos(pitch), cos(roll)/cos(pitch)];

            PQR = Ew\[rolldot;pitchdot;yawdot];
            P = PQR(1);
            Q = PQR(2);
            R = PQR(3);

            % See if optotrak dropped out and set Luenberger gain
            % accordingly
            if abs(x) > 10^5
                L = 0; % i.e. ignore current state est. and use model only
            else
                L = 0.2; % Luenberger gain
            end

            if s_last(1) > 0.8 % (If plane has left launcher)
               % Do prediction with model
               sdot_pred = p.dynamics(0,s_last,u_last);
               s_new = s_last + (t-t_last)*(sdot_pred') + L*([x;y;z;roll;pitch;yaw;U;V;W;P;Q;R] - s_last);
               if abs(x) < 10^5 % Optotrak has it
                s_new = [x;y;z;roll;pitch;yaw;s_new(7:12)]; % Use x,y,z, etc. from optotrak (i.e. only estimate velocities)
               end
            else  % (Otherwise don't use model for prediction)
               s_new = [x;y;z;roll;pitch;yaw;U;V;W;P;Q;R];
            end
            
            statemsg.positions_dot = [U;V;W];
            statemsg.angles_dot = [P;Q;R];
        else
            s_new = zeros(12,1);
        end
        
        % Save these state estimates for use in next iteration
        s_last = s_new;
        u_last = u;
        t_last = t;
        
        
        statemsg.positions = [s_new(1);s_new(2);s_new(3)]; % x,y,z
        statemsg.angles = [s_new(4);s_new(5);s_new(6)]; % roll, pitch, yaw
        statemsg.timestamp = msg.timestamp;

        % Publish state message
        lc.publish('wingeron_xhat', statemsg);
        
    end
    
    
    
    

end

            
