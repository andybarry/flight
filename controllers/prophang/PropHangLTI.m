classdef PropHangLTI < Control
    % Implements LTI control for prophang on the wingeron aircraft.
    %
    % Usage: <pre>runLCMControl(ProphangLTI, WingeronLCMCoder);</pre>
    
    properties 
        
        gainMatrixStorage; % stores the gain matrix messages
        ltiObj; % LTIControl object
        
    end
  methods
    function obj = PropHangLTI()
      obj = obj@Control(12,4);
      
      obj.gainMatrixStorage = LCMStorage('wingeron_gains');
      obj.ltiObj = LTIControl(zeros(12,1), zeros(4,1), zeros(12));
      
    end

    function u = control(obj,t,x)
        % Implements control function.  You shouldn't run this; instead
        % use: <pre>runLCMControl(ProphangLTI, WingeronLCMCoder);</pre>
        % 
        %
        %
        
        % get parameters
        gainMsg = obj.gainMatrixStorage.GetLatestMessage();
        
        % decode gainMsg
        [K, killed] = decodeGainMsg(gainMsg);
        
        if (killed == false && obj.gainMatrixStorage.storage_struct.was_killed == true)
            obj.gainMatrixStorage.storage_struct.was_killed = false;

            obj.gainMatrixStorage.storage_struct.x0 = x;

            obj.gainMatrixStorage.storage_struct.sin_t = 0;
            obj.gainMatrixStorage.storage_struct.sin_freq = sinFreqStart;
        end
        
        
        x0 = obj.gainMatrixStorage.storage_struct.x0;
        
        % update LTI control object
        obj.ltiObj.K = K;
        obj.ltiObj.x0 = x0;
        
        
        % get the control values
      
        u = obj.ltiObj.outupt(0,0,x);
        
        if (any(isnan(x)) || min(x) < -1e25)
            if (killed == 1)
                u(1) = 0;
                obj.gainMatrixStorage.storage_struct.was_killed = true;
            end
        end
      
        u = min(255, max(0, round(u)) );
        
        %TODO: send an LCM message with x0 in it!
        
        
        
        
        
        

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
    
    
    function [K, killed] = decodeGainMsg(gainMsg)
        % decodes the LCM message for the gains
        %
        % @param gainMsg LCM message to decode
        % @retval K gain matrix
        % @retval killed true if system was killed. You should
        % <b>respect this value</b>!
       
        K = gainMsg.K;
        killed = gainMsg.killed;
        
        
        
        
    end
    
  end
  
end