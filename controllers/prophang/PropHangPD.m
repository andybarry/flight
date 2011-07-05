classdef PropHangPD < LTIControl
    
    
    properties 
        was_killed = 1;
        xzero = 0.22;
        yzero = -0.3;
        zzero = -2.7;kk
      
        k;
        sin_t = 0;
        sin_freq = 60;
        
    end
  methods
    function obj = PropHangPD()
      %obj = obj@LTIControl(12,4);
      obj = obj@LTIControl(zeros(12,1), zeros(4,1), zeros(4,12));
            
    end

    function uOut = control(obj,t,x)
      sinFreqStart = 60;
      %x_zero = [0;0;-2.7;pi/2;0;0;zeros(6,1)]; ---> russ's version
      
      %x_zero = [0.22; -0.3; -2.7; pi/2; 0; 0; zeros(6,1)];
      %uOut_failure = [100 89 146 86];
      uOut_failure = [0 89 146 86];
      try
          load /var/www/gains.txt
          load /var/www/trim_nums.txt
      catch exception
          %uOut = [85 70 80 160];
          
          uOut = uOut_failure;
          disp('load file failure');
          return;
      end
      
      % check for a bad gain matrix
      if (length(gains) < 13)
          disp('Warning: Gain matrix not long enough!  Failing.');
          u = uOut_failure;
          uOut = [u kp_height kd_height kp_yaw kd_yaw kp_pitch kd_pitch kp_roll kd_roll kp_x kd_x kp_z kd_z obj.xzero obj.yzero obj.zzero];
          return;
      end
      
      %x_zero = [0.22; -0.3; -2.7; pi/2+0.05; 0; 0; zeros(6,1)];
      
      %x_zero = [0.22; -0.3; -2.7; pi/2+0.05; 0; 0; zeros(6,1)];
      
      
      
      
      
      %u_trim = 127*ones(4,1); -- russ's version
      %u_trim = [85 70 80 160];
      u_trim = trim_nums;
      u = u_trim;
      
      kp_height = gains(1);
      kd_height = gains(2);
      
      kp_yaw = gains(3);
      kd_yaw = gains(4);
      
      kp_pitch = gains(5);
      kd_pitch = gains(6);
      
      kp_roll = gains(7);
      kd_roll = gains(8);
      
      kp_x = gains(10);
      kd_x = gains(11);
      
      kp_z = gains(12);
      kd_z = gains(13);
      
      if (any(isnan(x)) || min(x) < -1e25)
          if (gains(9) == 1)
            u(1) = 0;
            obj.sin_freq = sinFreqStart;
            obj.was_killed = 1;

          end
          
          uOut = [u kp_height kd_height kp_yaw kd_yaw kp_pitch kd_pitch kp_roll kd_roll kp_x kd_x kp_z kd_z obj.xzero obj.yzero obj.zzero];
          return
      end
      
      if (gains(9) == 0 && obj.was_killed == 1)
          obj.was_killed = 0;
          obj.xzero = x(1);
          obj.yzero = x(2);
          obj.zzero = x(3);
          
          obj.sin_t = 0;
          obj.sin_freq = sinFreqStart;
      end
      
      %x_zero = [0.22; -0.3; -2.7; pi/2+0.05; 0; -0.4; zeros(6,1)];
      x_zero = [obj.xzero; obj.yzero; obj.zzero; 0; 0; 0; zeros(6,1)];
      % i dont think you should use this one x_zero = [obj.yzero; pi/2+0.05; 0; -0.4; zeros(4,1)];
      % x:
      % [x y z yaw pitch roll xdot ydot zdot yawdot pitchdot rolldot]'
      % u:
      % [throttle rudder elevator aileron]'
      
      % x is "left right" (setpoint: .22)
      % y is "up/down" (setpoint: -.49)
      % z is close/far" from the sensor (setpoint: -2.6)
      

      x = x - x_zero;
      
      %disp(x)
      
      
      %obj.k = [ kp_yaw kp_pitch kp_roll rp_height ];
      
      
       %u(1) = -kp_height*x(3) - kd_height*x(9);
       u(1) = -kp_height*x(2) - kd_height*x(8);
       
       u(2) = -kp_yaw*x(4) - kd_yaw*x(10);
       
       u(3) = -kp_pitch*x(5) - kd_pitch*x(11);
       
       u(4) = -kp_roll*x(6) - kd_roll*x(12);

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
      
    end
    
  end
  
end