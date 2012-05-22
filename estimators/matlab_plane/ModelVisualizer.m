classdef ModelVisualizer < Visualizer
% Implements the draw function for the MURI Plane 

  properties
      angleRep = 'euler';
  end

  methods
    function obj = ModelVisualizer(angleRep)
      obj = obj@Visualizer(1);
      obj.playback_speed = 1;
      obj.display_dt = 0;
      if (nargin > 0)
          obj.angleRep = angleRep;
      end
    end
    
    function draw(obj,t,x,u)
        persistent hFig;

        if (isempty(hFig))
            hFig = sfigure(25);
            set(hFig,'DoubleBuffer', 'on');
        end
        
        if nargin < 4
            u = zeros(4,1);
        end
        
        %Airplane definition
        hull_scale=0.04;
        air_hull=[1 3; -1 3; -1 1; -6 1; -6 -1; -1 -1; -0.8 -4.8; -3 -5; -3 -6;...
            3 -6; 3 -5; 0.8 -4.8; 1 -1; 6 -1; 6 1; 1 1; 1 3];
        air_hull=hull_scale*air_hull;
        %x_coll = [air_hull(:,1),air_hull(:,2),zeros(size(air_hull,1),1)];
        x_coll = [air_hull(:,2),air_hull(:,1),zeros(size(air_hull,1),1)];
        
        roll = x(4);
        pitch = x(5);
        yaw = x(6);
        
        Rotx = [1 0 0; 0 cos(roll) sin(roll); 0 -sin(roll) cos(roll)];
        Roty = [cos(pitch) 0 -sin(pitch); 0 1 0; sin(pitch) 0 cos(pitch)];
        Rotz = [cos(yaw) sin(yaw) 0; -sin(yaw) cos(yaw) 0; 0 0 1];
        GtoB = Rotx*Roty*Rotz;
        
        % Size of the arena.  
        %Important that it is 8 wide in the y-direction
        X_LOW = -2 + x(1);
        X_HIGH = 6 + x(1);
        Y_LOW = -4 + x(2);
        Y_HIGH = 4 + x(2);
        Z_LOW = -4 + x(3);
        Z_HIGH = 4 + x(3);
        if x(2)<-3
            Yoff = -3;
        elseif x(2)>3
            Yoff = 3;
        else
            Yoff = x(2);
        end
        if x(3)<-3
            Zoff = -3;
        elseif x(3)>3
            Zoff = 3;
        else
            Zoff = x(3);
        end
        Y_LOW = Y_LOW - Yoff;
        Y_HIGH = Y_HIGH - Yoff;
        Z_LOW = Z_LOW - Zoff;
        Z_HIGH = Z_HIGH - Zoff;
        curr_window = [X_LOW,X_HIGH,Y_LOW,Y_HIGH,Z_LOW,Z_HIGH];

        axis_size = curr_window;
        %Velocity Bar:
        velz = [Z_LOW, Z_LOW, .2*x(7) + Z_LOW,.2*x(7) + Z_LOW];
        velx = X_LOW*ones(1,4);
        vely = [Y_HIGH, Y_HIGH-1, Y_HIGH-1, Y_HIGH];
        %P (roll angular rate) bar
        Px = X_LOW*ones(1,4);
        Py = [Y_HIGH-1, Y_HIGH-2, Y_HIGH-2, Y_HIGH-1];
        Pz = [Z_LOW+1, Z_LOW+1, .2*x(10) + Z_LOW+1,.2*x(10) + Z_LOW+1];
        if (abs(x(10))>2*pi)
            Pcolor = 'red';
        else
            Pcolor = 'green';
        end
        %P (pitch angular rate) bar
        Qx = X_LOW*ones(1,4);
        Qy = [Y_HIGH-2, Y_HIGH-3, Y_HIGH-3, Y_HIGH-2];
        Qz = [Z_LOW+1, Z_LOW+1, .2*x(11) + Z_LOW+1,.2*x(11) + Z_LOW+1];
        if (abs(x(11))>2*pi)
            Qcolor = 'red';
        else
            Qcolor = 'green';
        end
        %R (yaw angular rate) bar
        Rx = X_LOW*ones(1,4);
        Ry = [Y_HIGH-3, Y_HIGH-4, Y_HIGH-4, Y_HIGH-3];
        Rz = [Z_LOW+1, Z_LOW+1, .2*x(12) + Z_LOW+1,.2*x(12) + Z_LOW+1];
        if (abs(x(12))>2*pi)
            Rcolor = 'red';
        else
            Rcolor = 'green';
        end
        %Thrust bar
        tx = X_LOW*ones(1,4);
        ty = [Y_HIGH-4, Y_HIGH-5, Y_HIGH-5, Y_HIGH-4];
        tz = [Z_LOW, Z_LOW, 2*u(1)/255 + Z_LOW,2*u(1)/255 + Z_LOW];
        if (u(1)>150)
            tcolor = 'red';
        else
            tcolor = 'blue';
        end
        %elev input bar
        elevx = X_LOW*ones(1,4);
        elevy = [Y_HIGH-5, Y_HIGH-6, Y_HIGH-6, Y_HIGH-5];
        elevz = [Z_LOW+1, Z_LOW+1, u(2)/20 + Z_LOW+1,u(2)/20 + Z_LOW+1];
        if (abs(x(12))>2*pi)
            elevcolor = 'red';
        else
            elevcolor = 'blue';
        end
        rudx = X_LOW*ones(1,4);
        rudy = [Y_HIGH-6, Y_HIGH-7, Y_HIGH-7, Y_HIGH-6];
        rudz = [Z_LOW+1, Z_LOW+1, u(3)/20 + Z_LOW+1,u(3)/20 + Z_LOW+1];
        if (abs(x(12))>2*pi)
            rudcolor = 'red';
        else
            rudcolor = 'blue';
        end
        ailx = X_LOW*ones(1,4);
        aily = [Y_HIGH-7, Y_HIGH-8, Y_HIGH-8, Y_HIGH-7];
        ailz = [Z_LOW+1, Z_LOW+1, u(4)/20 + Z_LOW+1,u(4)/20 + Z_LOW+1];
        if (abs(x(12))>2*pi)
            ailcolor = 'red';
        else
            ailcolor = 'blue';
        end
      %Plot the airplane
      if(~isempty(x))
          plane_color=[0, 0, 0];
          [x_points,y_points,z_points]=plane_coll_points(x,x_coll,obj.angleRep);
          fill3(x_points+x(1),-y_points-x(2),-z_points+x(3),plane_color);
          %velocity bar
      end
      hold on
      fill3(velx, vely, velz, 'green');
      fill3(Px, Py, Pz, Pcolor);
      fill3(Qx, Qy, Qz, Qcolor);
      fill3(Rx, Ry, Rz, Rcolor);
      fill3(tx, ty, tz, tcolor);
      fill3(elevx, elevy, elevz, elevcolor);
      fill3(rudx, rudy, rudz, rudcolor);
      fill3(ailx, aily, ailz, ailcolor);
      hold off
      title(strcat('t = ', num2str(t,'%.5f')));
      axis(axis_size);
      set(gca,'DataAspectRatio',[1 1 1]);
      grid on;
      ylabel('Vel,P,Q,R,thr,elev,rud,ail')
      drawnow
      %xlabel('x');
      %ylabel('z');
      %zlabel('y');
    end    
  end
end


function [x_out,y_out,z_out]=plane_coll_points(x,x_coll,angleRep)

if strcmp(angleRep,'euler')
    % Make sure size of x is correct
%     if (length(x) ~= 2*6)
%         error('x should contain 12 elements: x,y,z,yaw,pitch,roll');
%     end
    %keyboard;
    angs = [-x(4);-x(5);-x(6)];
    %q = SpinCalc('EA321toQ',(180/pi)*angs', 0.001, 1);
    %points = qrot3d(x_coll,[q(end),q(1:3)]);
    %points = qrot3d(x_coll,q);
    M = SpinCalc('EA123toDCM',(180/pi)*angs',0.001,1);
    %keyboard;
    points = M*x_coll';
    points = points';

elseif strcmp(angleRep,'quaternion')
    % Make sure size of x is correct
    if (length(x) ~= 2*7)
        error('x should contain 14 elements: x,y,z,quaternion');
    end
    q = [x(4),x(5),x(6),x(7)]./(norm([x(4),x(5),x(6),x(7)]));
    points = qrot3d(x_coll,q);
else
    error('This angle representation is not supported');
end
    

x_out = points(:,1);
y_out = points(:,2);
z_out = points(:,3);

end









