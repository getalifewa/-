%% Establish Serial Connection
% Specify COM Port
% port = '/dev/cu.usbmodem1413101';                  % MacOS Port Structure - ONLY EXAMPLE, MAY NOT BE EXACT ADDRESS
port = 'COM3';                                 % Windows Port Structure - ONLY EXAMPLE, MAY NOT BE EXACT ADDRESS

% Setup Serial Connection
baudrate = 230400;
s = establishSerial(port, baudrate); 

% Read Connected Motor IDs
[numID, ID] = getMotorIDs(s)

% Set Change Motor ID flag - CHANGE TO ADJUST MOTOR ID
changeMotorID = false;

%% Sample Code for Reading Motor Feedback
% Read motor position feedback
[motorFB, eFB] = readFB(s, numID);
motorFB ;
motorFB/pi*180;
% Display motor feedback

%% Sample Code for Changing To Position Control Mode
% Set motor EPROM to position control mode
setControlMode(s, "position");

%% Sample Code for Changing To Velocity Control Mode
% Set motor EPROM to velocity control mode
setControlMode(s, "velocity");

%% Initialise Link Length of the Robot
l1 = 18;
l2 = 18;
l3 = 20;
l4 = 15;


%% Go to position
desitination = [24 0 15];

[q1 q2 q3 q4] = inverse(desitination(1),desitination(2),desitination(3));
% Set motor EPROM to position control mode
motorPos = [-q1 + 3*pi/180 q2+7*pi/180 q3+98*pi/180 q4-45*pi/180 55*pi/180];

[motorFB, eFB] = readFB(s, numID);

angle1 = linspace(motorFB(1),motorPos(1),50);
angle2 = linspace(motorFB(2),motorPos(2),50);
angle3 = linspace(motorFB(3),motorPos(3),50);
angle4 = linspace(motorFB(4),motorPos(4),50);

for i = 1:length(angle1)
    Pos = [angle1(i) angle2(i) angle3(i) angle4(i) 25*pi/180];
    sendJointPos(s, Pos, numID);
end

%% Part 2
% Calculate an equation of forward kinematics
syms Q1 Q2 Q3 Q4

Q4 = -Q2-Q3;

l1 = 18;
l2 = 18;
l3 = 20;
l4 = 15;


% Find the coordinate of the end effector in frame 0
T01 = translationMatrix(0, 0, l1, Q1);
T12 = translationMatrix(0, 90, 0, Q2);
T23 = translationMatrix(l2, 0, 0, Q3);
T34 = translationMatrix(l3, 0, 0, Q4);
T45 = translationMatrix(0, -90, 0, 0);
T5E = translationMatrix(0, 180, l4, 0);

T02 = T01*T12;
T03 = T01*T12*T23;
T04 = T01*T12*T23*T34;
T05 = T01*T12*T23*T34*T45;

T0E = T01*T12*T23*T34*T45*T5E;

rE = T0E(1:3,end);

X = rE(1);
Y = rE(2);
Z = rE(3);

% Calculate an equation for inversed jacobian
Z1 = [0;0;1];
Z2 = [sin(Q1);-cos(Q1);0];
Z3 = [sin(Q1);-cos(Q1);0];
Z4 = [sin(Q1);-cos(Q1);0];
Z5 = [0;0;1];

% Find the translation matrix Tab (from frame b to a)            

r14 = T04(1:3,4) - T01(1:3,4);
r24 = T04(1:3,4) - T02(1:3,4);
r34 = T04(1:3,4) - T03(1:3,4);

J = [cross(Z1,r14) cross(Z2,r24) cross(Z3,r34)];
Jinv = inv(J);

% Set to velocity mode
setControlMode(s, "velocity");

x_ref = 24;
y_ref = 0;
z_ref = 15;

KP_x = 1;
KP_y = 0;
KP_z = 1.3;

KI_x = 0.1;
KI_y = 0.1;
KI_z = 0.1;

KP = diag([KP_x KP_y KP_z]);

KI = diag([KI_x KI_y KI_z]);

eSum = 0;
Ts = 0;

%for i = 1:500
i = 11;
    tStart = tic;
    [motorFB, eFB] = readFB(s, numID);
 
    motorFB = motorFB * 180 /pi;
    motorFB(1) = - (motorFB(1) - 3);
    motorFB(2) = motorFB(2) - 7;
    motorFB(3) = motorFB(3) - 98;
    motorFB(4) = motorFB(4) + 45;

    x = subs(X, [Q1 Q2 Q3 Q4], [motorFB(1) motorFB(2) motorFB(3) motorFB(4)]);
    y = subs(Y, [Q1 Q2 Q3 Q4], [motorFB(1) motorFB(2) motorFB(3) motorFB(4)]);
    z = subs(Z, [Q1 Q2 Q3 Q4], [motorFB(1) motorFB(2) motorFB(3) motorFB(4)]);
    
    e = [x_ref - x; y_ref - y; z_ref - z];
    eSum = eSum + Ts*e;
    
    Jaco = subs(Jinv, [Q1 Q2 Q3], [motorFB(1) motorFB(2) motorFB(3)]);
    
    Q_dot = double(Jaco * (KP * e));
    sendJointVel(s, [Q_dot'/180*pi 0 0], numID);
    Ts = toc(tStart);
%end

%% Task2 (b)

% Calculate an equation of forward kinematics
syms Q1 Q2 Q3 Q4

Q4 = -Q2-Q3;

l1 = 18;
l2 = 18;
l3 = 20;
l4 = 15;

R = [cosd(45) -sind(45) 0; sind(45) cosd(45) 0; 0 0 1];

% Find the coordinate of the end effector in frame 0
Tr0 = [R [0;0;0];0 0 0 1];
T01 = translationMatrix(0, 0, l1, Q1);
T12 = translationMatrix(0, 90, 0, Q2);
T23 = translationMatrix(l2, 0, 0, Q3);
T34 = translationMatrix(l3, 0, 0, Q4);
T45 = translationMatrix(0, -90, 0, 0);
T5E = translationMatrix(0, 180, l4, 0);

Tr1 = Tr0*T01;
Tr2 = Tr0*T01*T12;
Tr3 = Tr0*T01*T12*T23;
Tr4 = Tr0*T01*T12*T23*T34;
Tr5 = Tr0*T01*T12*T23*T34*T45;

TrE = Tr0*T01*T12*T23*T34*T45*T5E;

rE = R*T0E(1:3,end);

X = rE(1);
Y = rE(2);
Z = rE(3);

% Calculate an equation for inversed jacobian
Z1 = R*[0;0;1];
Z2 = R*[sin(Q1);-cos(Q1);0];
Z3 = R*[sin(Q1);-cos(Q1);0];
Z4 = R*[sin(Q1);-cos(Q1);0];
Z5 = R*[0;0;1];

% Find the translation matrix Tab (from frame b to a)            
r14 = Tr4(1:3,4) - Tr1(1:3,4);
r24 = Tr4(1:3,4) - Tr2(1:3,4);
r34 = Tr4(1:3,4) - Tr3(1:3,4);

J = [cross(Z1,r14) cross(Z2,r24) cross(Z3,r34)];
Jinv = inv(J);

% Set to velocity mode
setControlMode(s, "velocity");

x_ref = 24;
y_ref = 0;
z_ref = 15;

KP_x = 0;
KP_y = 1;
KP_z = 1.3;

KP = diag([KP_x KP_y KP_z]);

KI = diag([KI_x KI_y KI_z]);

eSum = 0;
Ts = 0;

for i = 1:500
    tStart = tic;
    [motorFB, eFB] = readFB(s, numID);
 
    motorFB = motorFB * 180 /pi;
    motorFB(1) = - (motorFB(1) - 3);
    motorFB(2) = motorFB(2) - 7;
    motorFB(3) = motorFB(3) - 98;
    motorFB(4) = motorFB(4) + 45;

    x = subs(X, [Q1 Q2 Q3 Q4], [motorFB(1) motorFB(2) motorFB(3) motorFB(4)]);
    y = subs(Y, [Q1 Q2 Q3 Q4], [motorFB(1) motorFB(2) motorFB(3) motorFB(4)]);
    z = subs(Z, [Q1 Q2 Q3 Q4], [motorFB(1) motorFB(2) motorFB(3) motorFB(4)]);
    
    e = [x_ref - x; y_ref - y; z_ref - z];
    eSum = eSum + Ts*e;
    
    Jaco = subs(Jinv, [Q1 Q2 Q3], [motorFB(1) motorFB(2) motorFB(3)]);
    
    Q_dot = double(Jaco * (KP * e));
    sendJointVel(s, [Q_dot'/180*pi 0 0], numID);
    Ts = toc(tStart);
end

%% Task 2(c)

% Calculate an equation of forward kinematics
syms Q1 Q2 Q3 Q4

Q4 = -Q2-Q3;

l1 = 18;
l2 = 18;
l3 = 20;
l4 = 15;


% Find the coordinate of the end effector in frame 0
T01 = translationMatrix(0, 0, l1, Q1);
T12 = translationMatrix(0, 90, 0, Q2);
T23 = translationMatrix(l2, 0, 0, Q3);
T34 = translationMatrix(l3, 0, 0, Q4);
T45 = translationMatrix(0, -90, 0, 0);
T5E = translationMatrix(0, 180, l4, 0);

T02 = T01*T12;
T03 = T01*T12*T23;
T04 = T01*T12*T23*T34;
T05 = T01*T12*T23*T34*T45;

T0E = T01*T12*T23*T34*T45*T5E;

rE = T0E(1:3,end);

X = rE(1);
Y = rE(2);
Z = rE(3);

% Calculate an equation for inversed jacobian
Z1 = [0;0;1];
Z2 = [sin(Q1);-cos(Q1);0];
Z3 = [sin(Q1);-cos(Q1);0];
Z4 = [sin(Q1);-cos(Q1);0];
Z5 = [0;0;1];

% Find the translation matrix Tab (from frame b to a)            

r14 = T04(1:3,4) - T01(1:3,4);
r24 = T04(1:3,4) - T02(1:3,4);
r34 = T04(1:3,4) - T03(1:3,4);

J = [cross(Z1,r14) cross(Z2,r24) cross(Z3,r34)];
Jinv = inv(J);

% Set to velocity mode
setControlMode(s, "velocity");

x_ref = 24;
y_ref = 0;
z_ref = 15;

KP_x = 1;
KP_y = 0;
KP_z = 1.3;

KP = diag([KP_x KP_y KP_z]);

eSum = 0;
Ts = 0;

for i = 1:500
    tStart = tic;
    [motorFB, eFB] = readFB(s, numID);
 
    motorFB = motorFB * 180 /pi;
    motorFB(1) = - (motorFB(1) - 3);
    motorFB(2) = motorFB(2) - 7;
    motorFB(3) = motorFB(3) - 98;
    motorFB(4) = motorFB(4) + 45;

    x = subs(X, [Q1 Q2 Q3 Q4], [motorFB(1) motorFB(2) motorFB(3) motorFB(4)]);
    y = subs(Y, [Q1 Q2 Q3 Q4], [motorFB(1) motorFB(2) motorFB(3) motorFB(4)]);
    z = subs(Z, [Q1 Q2 Q3 Q4], [motorFB(1) motorFB(2) motorFB(3) motorFB(4)]);
    
    e = [x_ref - x; y_ref - y; z_ref - z];
    eSum = eSum + Ts*e;
    
    if abs(e(2)) >= 7
        e(2) = e(2)-7;
        Jaco = subs(Jinv, [Q1 Q2 Q3], [motorFB(1) motorFB(2) motorFB(3)]);
        Q_dot = double(Jaco * (KP * e));
        sendJointVel(s, [Q_dot'/180*pi 0 0], numID);
        Ts = toc(tStart);
    end
end

