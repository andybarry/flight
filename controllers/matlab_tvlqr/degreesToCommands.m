function umsg = degreesToCommands(umsg)
    % Converts airplane degree deflections to servo commands (assumes that
    % you'll add in the trims later)
    %
    % @param umsg message to convert in the style of the LCM messages (as a
    %   structure)
    %   <pre>
    %     rudder = umsg.rudder;
    %     elevator = umsg.elevator;
    %     aileronL = umsg.aileronLeft;
    %     aileronR = umsg.aileronRight;
    % </pre>
    %
    % @retval umsg converted structure

    rudder = umsg.rudder;
    elevator = umsg.elevator;
    aileronL = umsg.aileronLeft;
    aileronR = umsg.aileronRight;

    umsg.elevator = elevator/(-0.3367);
    umsg.rudder = rudder/0.3786;
    umsg.aileronLeft = aileronL/0.4699;
    umsg.aileronRight = aileronR/0.4820;
    
end