classdef WingeronLCMCoder < LCMCoder
% Encodes and Decodes pendulum-specific LCM smessages 

  properties
    
  end

  methods
    function obj = WingeronLCMCoder
      obj = obj@LCMCoder(12,5,0);
    end
    
    function str = getRobotName(obj)
      % robot name
      str = 'wingeron';
    end
    
    %{
    function msg = encodeX(obj,t,x)
      % encodes the state message
      msg = lcmtypes.lcmt_hotrod_optotrak();
      msg.timestamp = t;
      msg.positions = x(1:3)*1000;
      msg.angles = x(4:6);
      
    end
    
    
    function [x,t] = decodeX(obj,msg)

      % decodes the state message
      msg = lcmtypes.lcmt_hotrod_optotrak(msg.data);
      
      t = msg.timestamp;
      
      % the yaw wraps at a really annoying place, so
      % we change the wrap point here
      % be careful of using mod() because the numbers
      % might not be exactly what you expect
      if (msg.angles(3) < 0)
          yaw = msg.angles(3) + pi;
      else
          yaw = msg.angles(3) - pi;
      end
      
      pitch = msg.angles(1);
      roll = msg.angles(2);
      
      x = [msg.positions/1000; yaw; pitch; roll];
      
      x = double(x);
      
    end
    %}
    
    function msg = encodeX(obj,t,x)
%       % encodes the input message
       msg = lcmtypes.lcmt_optotrak_xhat();
       msg.timestamp = t;
       msg.positions = xhat(1:3);
       msg.angles = xhat(4:6);
       
       msg.positions_dot = xhat(7:9);
%       msg.positions_dot = xhat(7:8);
       msg.angles_dot = xhat(10:12);
       

%        msg.xhat = xhat';
        
    end
    
    function [xhat,t] = decodeX(obj,msg)
      % decodes the state message
       msg = lcmtypes.lcmt_optotrak_xhat(msg.data);
       
       t = msg.timestamp;
       
       if (any(isnan(msg.positions)))
           xhat = NaN*ones(12,1);
%            xhat = NaN*ones(8,1);
           return
       end
      
       xhat = [msg.positions; msg.angles];
       xdot = [msg.positions_dot; msg.angles_dot];
       
       xhat = [xhat;xdot];

%        xhat = msg.xhat;
       
      
    end
    
    function msg = encodeU(obj,t,u)
      % encodes the input message
      msg = lcmtypes.lcmt_wingeron_u();
      msg.timestamp = t;
      msg.throttleFront = u(1);
      msg.throttleRear = u(1);
      msg.rudder = u(2);
      msg.elevator = u(3);
      msg.aileronLeft = u(4);
      msg.aileronRight = u(5);
      
    end
    
    function [u,t] = decodeU(obj,msg)
      % decodes the input message
      msg = lcmtypes.lcmt_wingeron_u(msg.data);
      u = [msg.throttle; msg.rudder; msg.elevator; msg.aileronLeft; msg.aileronRight];
      t = msg.timestamp;
    end

    function msg = encodeY(obj, t, u)
        msg = [];
    end

    function y = decodeY(obj, msg)
        y = 0;
    end
  end
  
  
end
