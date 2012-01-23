classdef PropHangLTI < SmoothRobotLibSystem
    % Implements LTI control for prophang on the wingeron aircraft.
    %
    % Usage: <pre>runLCMControl(ProphangLTI, WingeronLCMCoder);</pre>
    
    properties 
        
        gainMatrixStorage; % stores the gain matrix messages
        ltiObj; % LTIControl object
        
        x0lcm; % lcm object for sending data about x0
        
    end
    
  methods
    function obj = PropHangLTI()
        % constructor for PropHangLTI
        obj = obj@SmoothRobotLibSystem(0, 0, 12, 5);

        obj.gainMatrixStorage = LCMStorage('wingeron_gains');
        
        obj.gainMatrixStorage.storage_struct.x0 = zeros(12,1);
        obj.gainMatrixStorage.storage_struct.was_live = false;
        obj.gainMatrixStorage.storage_struct.t0 = 0;
        
        obj.ltiObj = LTIControl(zeros(12,1), zeros(4,1), zeros(12));
        
        obj.x0lcm = lcm.lcm.LCM.getSingleton();
    end

    function u = output(obj,t,junk,x)
        % Implements control function.  You shouldn't run this; instead
        % use: <pre>runLCMControl(ProphangLTI, WingeronLCMCoder);</pre>
        % 
        % @param t time
        % @param x state vector
        %
        % @retval u control action vector
        
        % get parameters
        gainMsg = obj.gainMatrixStorage.GetLatestMessage();
        
        % decode gainMsg
        [K, u0, live] = obj.decodeGainMsg(gainMsg);
        
        if (live == true && obj.gainMatrixStorage.storage_struct.was_live == false)
            obj.gainMatrixStorage.storage_struct.was_live = true;

            obj.gainMatrixStorage.storage_struct.x0 = [x(1); x(2); x(3); zeros(9,1)];
            
            % send LCM message logging the change in x0
            
            x0msg = lcmtypes.lcmt_wingeron_x0();
            x0msg.x0 = obj.gainMatrixStorage.storage_struct.x0;
            x0msg.timestamp = t;
            
            obj.x0lcm.publish('wingeron_x0',x0msg);
            
            obj.gainMatrixStorage.storage_struct.sin_t = 0;
            %obj.gainMatrixStorage.storage_struct.sin_freq = sinFreqStart;
        end
        
        
        x0 = obj.gainMatrixStorage.storage_struct.x0;
        
        % update LTI control object
        obj.ltiObj.K = K;
        obj.ltiObj.x0 = x0;
        obj.ltiObj.u0 = u0;
        
        
        % get the control values
      
        u = obj.ltiObj.output(0,0,x);
        
        if (any(isnan(x)) || min(x) < -1e25)
            u = obj.ltiObj.u0;
        end
        
        if (live ~= true)
            u(1) = 0;
            u(3) = 150; % save the plane from elevator damage by slamming the elevator over on kill
            obj.gainMatrixStorage.storage_struct.was_live = false;
        end
        
        if (obj.gainMatrixStorage.storage_struct.t0 <= 0)
            obj.gainMatrixStorage.storage_struct.t0 = t;
        end
        
        sin_t = (t-obj.gainMatrixStorage.storage_struct.t0) / 1000;
        
        % add disturbances
        sin_freq1 = 4;
        sin_freq2 = 5;
        sin_freq3 = 7;
        sin_freq4 = 9;

        %addterm = sin(obj.sin_t * obj.sin_freq);
        addterm1 = sin(sin_t * sin_freq1); %disabled
        addterm2 = sin(sin_t * sin_freq2);
        addterm3 = sin(sin_t * sin_freq3);
        addterm4 = sin(sin_t * sin_freq4);%%%%disabled


        addterm = addterm1 + addterm2 + addterm3 + addterm4;
        
        %addterm = addterm1 + addterm2 + addterm3;

        %u(2) = u(2) + addterm * 7; % rudder
        %u(3) = u(3) + addterm * 4; % elevator
        u(4) = u(4) + addterm * 13; % aileron
        u(5) = u(5) + addterm * 13; % aileron
        
%}      
        
        
        % ---- do NOT add code past this line -----
        % min/max protection here
        
        u = min(255, max(0, round(u)) );
        
    end
    
  end
  
  methods(Static)
      
    function [K, u0, live] = decodeGainMsg(msg)
        % decodes the LCM message for the gains and converts it to a gain
        % matrix
        %
        % @param msg LCM message to decode
        % @retval K gain matrix
        % @retval u0 array of trim values (x0 = [trimThrottle, trimRudder,
        % trimElevator, trimAileronLeft, trimAileronRight]')
        % @retval live true if system is live, false if killed. You should
        % <b>respect this value</b> and <b>kill</b> the system when false.
        %
        % state vector = [ x y z yaw pitch roll xdot ydot zdot yawdot pitchdot rolldot]'
        %
        % K has 5 rows corresponding to each output:
        %   row 1: throttle
        %   row 2: rudder
        %   row 3: elevator
        %   row 4: aileronLeft
        %   row 5: aileronRight
        
        msg = lcmtypes.lcmt_wingeron_gains(msg.data);
        
        msg.p_rudder = -msg.p_rudder;
        msg.d_rudder = -msg.d_rudder;
        
        msg.p_elevator = -msg.p_elevator;
        msg.d_elevator = -msg.d_elevator;
        
        msg.p_aileron = -msg.p_aileron;
        msg.d_aileron = -msg.d_aileron;
        
        p_throttle = msg.p_throttle / 1000;
        d_throttle = msg.d_throttle / 1000;
        
        p_x = msg.p_x / 1000;
        p_z = -msg.p_z / 1000;
        
        d_x = msg.d_x / 1000;
        d_z = -msg.d_z / 1000;
        
        
        K = [ 0, p_throttle,  zeros(1,5), d_throttle, zeros(1,4) ;
              p_x, 0, 0, msg.p_rudder, 0, 0, d_x, 0, 0, msg.d_rudder, 0, 0 ;
              0, 0, p_z, 0, msg.p_elevator, 0, 0, 0, d_z, 0, msg.d_elevator, 0 ;
              0, 0, 0, 0, 0, msg.p_aileron, 0, 0, 0, 0, 0, msg.d_aileron ;
              0, 0, 0, 0, 0, msg.p_aileron, 0, 0, 0, 0, 0, msg.d_aileron ; ];
            
        if (msg.live == 1)
            live = true;
        else
            live = false;
        end
        
        u0 = [ msg.trimThrottle, msg.trimRudder, msg.trimElevator, ...
                msg.trimAileronLeft, msg.trimAileronRight]';
        
    end

  end
  
end