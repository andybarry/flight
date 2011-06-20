classdef SquareWaveHotrodControl < Control

  methods
    function obj = SquareWaveHotrodControl()
      obj = obj@Control(12,4);

    end

    function u = control(obj,t,x)


      
      %uOut = [u kp_height kd_height kp_yaw kd_yaw kp_pitch kd_pitch kp_roll kd_roll obj.xzero obj.yzero obj.zzero];
      
      
      %u = [0; 70; 80; 160];
      u = [0; 128; 128; 128];
  
      %{
            msg.throttle = u(1);
      msg.rudder = u(2);
      msg.elevator = u(3);
      msg.aileron = u(4);
      %}
      
      % enable for square wave
      if (mod(t/1000,6) > 3)
          u(4) = 30;
      else
          u(4) = 150;
      end
      
      % enable for sin wave
      %u(3) = (sin(t/500) + 1) *128;
      
      %u(3) = u(3) * 0.686 + 40
      u = [u; zeros(20,1)];

    end
    
  end
  
end