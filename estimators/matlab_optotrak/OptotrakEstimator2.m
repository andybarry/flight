classdef OptotrakEstimator2 < Estimate
    
    
    properties
        filter_roll
        filter_yaw
        filter_pitch

        filter_x
        filter_y
        filter_z
        
        last_x
        last_t
        last_xhat = zeros(8,1);
        model
        L
    end
  methods
    function obj = OptotrakEstimator2()
      obj = obj@Estimate(9,12);
 
    end

    function xhat = estimate(obj,t,x,u)
      % decodes the state message
      
      x = double(x);
      
      positions = x(1:3);
      
      angles = x(4:end);
      
      % yaw is offset by +0.172
      angles(1) = angles(1) - 0.165;

      % pitch is offset by -0.05
      angles(2) = angles(2) + 0.05; 
      
      if (any(isnan(angles)))
          t = 0;
          xhat = NaN*ones(12,1);
          return
      end

      y = angles(1);
      p = angles(2);
      r = angles(3);

      %y = obj.filter_yaw.getvalue(y);
      %p = obj.filter_pitch.getvalue(p);
      %r = obj.filter_roll.getvalue(r);

      x = [positions; y; p; r];

      %x(1) = obj.filter_x.getvalue(x(1));
      %x(2) = obj.filter_y.getvalue(x(2));
      %x(3) = obj.filter_z.getvalue(x(3));



      %nump = 100;
      
      
      if (obj.last_t>0)
          %xdot = (x-obj.last_x)/(t-obj.last_t);
          % use fixed timestep because timestep coming in varies wildly
          xdot = (x-obj.last_x)*100;
%          scope('hotrod','xhat_vel',t,xdot(1),struct('scope_id',1,'num_points',nump));
%          scope('hotrod','xhat_post_diff',t,(x(1) - obj.last_x(1))*100,struct('scope_id',1,'num_points',nump,'linespec','r--'));
      else
          xdot = zeros(6,1);
      end
      
      %y = [x(1:3); x(5)];
      %xhat = obj.model.A * obj.last_xhat + obj.model.B * u' + obj.L * (y - obj.model.C*obj.last_xhat);

      
      
     
      % xhat
      % [yaw pitch roll y yawd pitchd rolld yd]
      
      
%       x_relavent = [x(1:3) x(5)];
%       xhat = obj.model.A * obj.last_xhat + obj.model.B * u + obj.L * (x_relavent - obj.last_xhat);
%      
%       % xhat
%       % [yaw pitch roll y yawd pitchd rolld yd]
%       
%       
%       xhat = [xhat(1:3) x(4) xhat(4) x(6) xhat(5:7) xdot(1) xhat(8) xdot(3)];
%       
        
      xhat = [x xdot];
      
      obj.last_x = x;
      obj.last_xhat = xhat;
      obj.last_t = t;
      
    end
    
  end
  
end