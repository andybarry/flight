classdef TVLQRControl < SmoothRobotLibSystem
    % TVLQR controller for airplane
    %
    % Usage: <pre>runLCMControl(TVLQRControl(...), WingeronLCMCoder);</pre>
    
    properties 
        


        gainMatrixStorage; % stores the gain matrix messages
        
        x0lcm; % lcm object for sending data about x0
        xtrigger; % trigger value to start the clock on the u-tape
        
        ltvcontrol; % LTVControl system for using TVLQR
        
        maxT; % Maximum time that the ltvcontrol runs for
        
        splineK1; % Spline of linear part of K (pre-computed for efficiency)
        splineK2; % Affine part
        
        u0spline; % Precomputed u0 trajectory
        delayT; % Time in seconds that we're using to account for delay.
    end
    
  methods
    function obj = TVLQRControl(ltvcontrol, xtrigger, delayT)
        % constructor for TVLQRControl
        %
        % @parma ltvcontrol LTVControl system
        % @param xtrigger x-value for the plane to cross to start the tape
        % @param delayT time in seconds to assume that the tape started
        %   before we detected it
        
        obj = obj@SmoothRobotLibSystem(0, 0, 12, 5);

        obj.gainMatrixStorage = LCMStorage('wingeron_gains');
        
        obj.gainMatrixStorage.storage_struct.x0 = zeros(12,1);
        obj.gainMatrixStorage.storage_struct.was_live = false;
        obj.gainMatrixStorage.storage_struct.t0 = 0;
        obj.gainMatrixStorage.storage_struct.t0_utape = 0;
        
        obj.maxT = ltvcontrol.K.tspan(end);
        
        obj.delayT = delayT;
        obj.xtrigger = xtrigger;
        
        obj.ltvcontrol = ltvcontrol;
        
        % precompute Ks from ltvcontrol
        disp('Precomputing K...');
        
        ts = ltvcontrol.K.getBreaks();
        K1 = zeros(5,12,length(ts));
        K2 = zeros(5,length(ts));
        j = 1;
        for t=ts
            thisK = ltvcontrol.K.eval(t);        
            K1(:,:,j) = thisK{1};
            K2(:,j) = thisK{2};
            j = j+ 1;
        end
        
        obj.splineK1 = spline(ts, K1);
        obj.splineK2 = spline(ts, K2);
        
        disp('Precomputing u0...');
        ts = ltvcontrol.u0.getBreaks();
        obj.u0spline = spline(ts,ltvcontrol.u0.eval(ts));
        
        
        disp('done.');
        
        
        obj.x0lcm = lcm.lcm.LCM.getSingleton();

    end

    function u = output(obj,t,junk,x)
        % Implements control function.  You shouldn't run this; instead
        % use: <pre>runLCMControl(TVLQRControl(pptraj, 1, ltvcontrol), WingeronLCMCoder);</pre>
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
        
        if (obj.gainMatrixStorage.storage_struct.t0 <= 0)
            obj.gainMatrixStorage.storage_struct.t0 = t;
        end
        
        if (obj.gainMatrixStorage.storage_struct.t0_utape <= 0 && x(1) > obj.xtrigger)
            obj.gainMatrixStorage.storage_struct.t0_utape = t;
            disp('u tape fire');
        end
        flagPastMax = false;
        
        % get the control values
        
        if obj.gainMatrixStorage.storage_struct.t0_utape < 0
            currentT = 0.1; % this is a hack to force MATLAB to run code now, instead of when the trajectory is actually fired
        else
            currentT = (t - obj.gainMatrixStorage.storage_struct.t0_utape)*86400; % matlab returns decimal days since 1900 (?!?!?!!)
            currentT = currentT + obj.delayT;
        end

        
        
         if (currentT > obj.maxT && obj.gainMatrixStorage.storage_struct.t0_utape > 0)
             % don't do feedback past the end of the tape because that
             % can cause the plane to flip out in the net and damage
             % itself.
             flagPastMax = true;

             u_from_tape = ppval(obj.u0spline, obj.maxT);
         else
             % normally, use feedback
            u_from_tape = ppval(obj.u0spline, currentT) - ppval(obj.splineK1,currentT)*(x - obj.ltvcontrol.x0.eval(currentT)) - ppval(obj.splineK2, currentT);
         end

        umsg = struct();
        umsg.throttle = u_from_tape(1);
        umsg.elevator = u_from_tape(2);
        umsg.rudder = u_from_tape(3);
        umsg.aileronLeft = u_from_tape(4);
        umsg.aileronRight = u_from_tape(5);

        umsg = degreesToCommands(umsg);

        u_from_tape = [ umsg.throttle; umsg.rudder; umsg.elevator; ...
            umsg.aileronLeft; umsg.aileronRight ];


        % check to see if we're actually going to use the things we
        % computed or if we're not past the launcher yet...
        if (obj.gainMatrixStorage.storage_struct.t0_utape <= 0)
           u_from_tape =  zeros(5,1);
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
      
        u(1) = min(180, u(1));
        
        
        % ---- do NOT add code past this line -----
        
        % No throttle after tape
        if (flagPastMax == true)
            u(1) = 0;
        end
        
        
        if (live ~= true)
            u(1) = 0;
            %u(3) = 150; % save the plane from elevator damage by slamming the elevator over on kill
            obj.gainMatrixStorage.storage_struct.was_live = false;
        end
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
        
%         msg.p_rudder = -msg.p_rudder;
%         msg.d_rudder = -msg.d_rudder;
%         
%         msg.p_elevator = -msg.p_elevator;
%         msg.d_elevator = -msg.d_elevator;
%         
%         msg.p_aileron = -msg.p_aileron;
%         msg.d_aileron = -msg.d_aileron;
%         
%         p_throttle = msg.p_throttle / 1000;
%         d_throttle = msg.d_throttle / 1000;
%         
%         p_x = msg.p_x / 1000;
%         p_z = -msg.p_z / 1000;
%         
%         d_x = msg.d_x / 1000;
%         d_z = -msg.d_z / 1000;
%         
        
%         K = [ 0, p_throttle,  zeros(1,5), d_throttle, zeros(1,4) ;
%               p_x, 0, 0, msg.p_rudder, 0, 0, d_x, 0, 0, msg.d_rudder, 0, 0 ;
%               0, 0, p_z, 0, msg.p_elevator, 0, 0, 0, d_z, 0, msg.d_elevator, 0 ;
%               0, 0, 0, 0, 0, msg.p_aileron, 0, 0, 0, 0, 0, msg.d_aileron ;
%               0, 0, 0, 0, 0, msg.p_aileron, 0, 0, 0, 0, 0, msg.d_aileron ; ];
        K = 0;
            
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
