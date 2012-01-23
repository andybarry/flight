classdef UtapeControl < SmoothRobotLibSystem
    % Plays back a utape for real time control
    %
    % Usage: <pre>runLCMControl(UtapeControl, WingeronLCMCoder);</pre>
    
    properties 
        
        gainMatrixStorage; % stores the gain matrix messages
        utape; % stores the tape to play back
        
        x0lcm; % lcm object for sending data about x0
        ztrigger; % trigger value to start the clock on the u-tape
    end
    
  methods
    function obj = UtapeControl(utape, ztrigger)
        % constructor for UtapeControl
        %
        % @param utape input trajectory (as a RobotLib Trajectory)
        % @param ztrigger z-value for the plane to cross to start the tape
        
        obj = obj@SmoothRobotLibSystem(0, 0, 12, 5);

        obj.gainMatrixStorage = LCMStorage('wingeron_gains');
        
        obj.gainMatrixStorage.storage_struct.x0 = zeros(12,1);
        obj.gainMatrixStorage.storage_struct.was_live = false;
        obj.gainMatrixStorage.storage_struct.t0 = 0;
        obj.gainMatrixStorage.storage_struct.t0_utape = 0;
        
        obj.utape = utape;
        obj.ztrigger = ztrigger;
        
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
        
        
        x0 = obj.gainMatrixStorage.storage_struct.x0;
        
        
%        if (any(isnan(x)) || min(x) < -1e25)
%            u = obj.ltiObj.u0;
%        end
        
        if (live ~= true)
            u(1) = 0;
            %u(3) = 150; % save the plane from elevator damage by slamming the elevator over on kill
            obj.gainMatrixStorage.storage_struct.was_live = false;
        end
        
        if (obj.gainMatrixStorage.storage_struct.t0 <= 0)
            obj.gainMatrixStorage.storage_struct.t0 = t;
        end
        
        if (obj.gainMatrixStorage.storage_struct.t0_utape <= 0 && x(3) < obj.ztrigger)
            obj.gainMatrixStorage.storage_struct.t0_utape = t;
        end
        
        
        % get the control values
        if (obj.gainMatrixStorage.storage_struct.t0_utape > 0)
            currentT = (t - obj.gainMatrixStorage.storage_struct.t0_utape)/1000

            u_from_tape = obj.utape.eval(currentT);

        else
           u_from_tape =  obj.utape.eval(0);
        end
        
        
        u = u_from_tape + u0; % add in the calibration/set point values
        
        
        %{
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