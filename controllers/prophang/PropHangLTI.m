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
            obj.gainMatrixStorage.storage_struct.was_live = false;
        end
      
        u = min(255, max(0, round(u)) );
        
        

        %{
        addterm = sin(obj.sin_t * obj.sin_freq);

        obj.sin_t = obj.sin_t + 0.01;
        obj.sin_freq = obj.sin_freq - 0.01;


        if (obj.sin_freq < 0.5)
        obj.sin_freq = 0.5;
        end


        u(2) = u(2) + addterm * 15;
        %u(3) = u(3) + addterm * 15;


        % add position control

        % x postion
        u(2) = u(2) - kp_x * x(1) - kd_x * x(7);

        % z position (evevator?)
        u(3) = u(3) - kp_z * x(3) - kd_z * x(9);
      
      
      %k = [5.81972081683635,3.98165427583584,-3.21284936974977,-4.88912166208106,-6.39505762333590,-0.609360622619920,-11.1597811094662,-2.46536195196173;0.444948880519526,-0.0197555924201845,0.431211112016844,0.305441062369966,-0.602288724304874,1.27154819412926,-0.562764487589815,-1.38346077264154;0.177884649664940,0.820774238488977,0.176032846476008,-1.56249846597632,2.63409576490163,-1.15508939135023,-2.44772678978091,0.446537152427191;0.492708574666643,0.912824542330009,-1.92844722643401,3.22999335168441,-1.71715343500542,-1.61352282795339,3.48822912340177,-1.03038033412878;];
      
%      u = -obj.k*x;
 
      u = min(254,max(0,round(u + u_trim)));
      
      % check kill
      if (gains(9) == 1)
          u(1) = 0;
          obj.sin_freq = 1;
          obj.was_killed = 1;
      end

      
      uOut = [u kp_height kd_height kp_yaw kd_yaw kp_pitch kd_pitch kp_roll kd_roll kp_x kd_x kp_z kd_z obj.xzero obj.yzero obj.zzero];
      
      %%% CHANGED FOR ONE EXPERIMENT CHANGE IT BACK
      %uOut = [u kp_height kd_height kp_yaw kd_yaw kp_pitch kd_pitch kp_roll u_trim(1) ];
      
      %disp(u)
      %}
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
        
        K = [ 0, msg.p_throttle,  zeros(1,5), msg.d_throttle, zeros(1,4) ;
              msg.p_x, 0, 0, msg.p_rudder, 0, 0, msg.d_x, 0, 0, msg.d_rudder, 0, 0 ;
              0, 0, msg.p_z, 0, msg.p_elevator, 0, 0, 0, msg.d_z, 0, msg.d_elevator, 0 ;
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