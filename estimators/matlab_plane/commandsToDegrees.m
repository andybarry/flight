function umsg = commandsToDegrees(zeros_msg,umsg)

throttle = umsg.throttleFront;
rudder = umsg.rudder;
elevator = umsg.elevator;
aileronL = umsg.aileronLeft;
aileronR = umsg.aileronRight;

umsg.elevator = elevator*(-0.3367) + 0.3367*zeros_msg.trimElevator;
umsg.rudder = rudder*0.3786 - 0.3786*zeros_msg.trimRudder;
umsg.aileronLeft = aileronL*0.4699 - 0.4699*zeros_msg.trimAileronLeft;
umsg.aileronRight = aileronR*0.4820 - 0.4820*zeros_msg.trimAileronRight;