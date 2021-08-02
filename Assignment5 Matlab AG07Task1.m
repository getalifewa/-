%% Establish Serial Connection

% Specify COM Port              % MacOS Port Structure - ONLY EXAMPLE, MAY NOT BE EXACT ADDRESS
port = 'COM3';                                 % Windows Port Structure - ONLY EXAMPLE, MAY NOT BE EXACT ADDRESS

% Setup Serial Connection
baudrate = 230400;
s = establishSerial(port, baudrate); 

% Read Connected Motor IDs
[numID, ID] = getMotorIDs(s)

% Set Change Motor ID flag - CHANGE TO ADJUST MOTOR ID
changeMotorID = false;


%% Sample Code for Reading Joystick Input

% Read joystick input
[xJoy, yJoy, eJoy] = readJoy(s);
[xJoy, yJoy]                                    % Display joystick input

%% Sample Code for Reading Motor Feedback

% Read motor position feedback
[motorFB, eFB] = readFB(s, numID);
motorFB 
motorFB/pi*180
% Display motor feedback

%% Sample Code for Changing To Position Control Mode

% Set motor EPROM to position control mode
setControlMode(s, "position");

%% Sample Code for Sending Position Commands - Motors Must Be In Position Control Mode

% Send basic speed up and slow down trajectory to motors
% for v = [0:0.01:1.8762 1.8762:-0.01:-1.8762 -1.8762:0.01:0]
%     
%      motorPos = zeros(1, numID) + v;
%     
%     % To control each motor individually change the above line to be in a format like this:
%     % motorVel = [q0 q1 q2 q3 q4 q5];
%     
%     sendJointPos(s, motorPos, numID);
% end

motorPos = [0*pi/180 7*pi/180 98*pi/180 -40*pi/180 55*pi/180];
sendJointPos(s, motorPos, numID);

%% Sample Code for Changing To Velocity Control Mode

% Set motor EPROM to velocity control mode
setControlMode(s, "velocity");
%% Sample Code for Sending Velocity Commands - Motors Must Be In Velocity Control Mode

% Send basic speed up and slow down trajectory to motors
for v = [0:0.01:pi pi:-0.01:-pi -pi:0.01:0]
    motorVel = zeros(1, numID) + v;
    
    % To control each motor individually change the above line to be in a format like this:
    % motorVel = [q0 q1 q2 q3 q4 q5];
    sendJointVel(s, motorVel, numID);
    
    % Delay
    for i = 1:100
    end
end


%% Sample Code for Setting Motor PID - Only Valid for Position Control Mode

% Set Motor internal PID controller. Gains must be integers.
tmpID = 1;
Kp = 20;
Ki = 5;
Kd = 9;
setPID(s, tmpID, Kp, Ki, Kd)


%% Change Motor ID (ONLY USE IF ABSOLUTELY NECESSARY - This should not be needed except for specific debugging scenarios)

% Change the ID number of a connected motor. NOTE: This should only be done with ONE MOTOR connected 
% at a time to ensure other IDs are not affected.


% ///////////////////////////////////////////////////// %
% /// This is the new ID you want the motor to have /// %

newID = 8;      % CHANGE THIS VALUE!    

% ///////////////////////////////////////////////////// %

if changeMotorID == true && numID == 1
    oldID = ID(1);  % This is the current ID of the motor you wish to change.
    changeID(s, oldID, newID);
    
    % Close serial connection. Establish Serial Connection section needs to be rerun.
    fclose(s);
    
    "ID Change Complete." 
    "Please re-run Establish Serial Connection section."
    
elseif changeMotorID == false
    fclose(s);
    "ID Change Failed. Please ensure flag is set to true!"
    "Please re-run Establish Serial Connection section."
elseif numID > 1
    fclose(s);
    "ID Change Failed. Please ensure flag is set to true and only one motor is connected!"
    "Please re-run Establish Serial Connection section."
end




%% Initialise Link Length of the Robot

l1 = 18;
l2 = 18;
l3 = 20;
l4 = 15;

%% Go to position

desitination = [17 0 10];

[q1 q2 q3 q4] = inverse(desitination(1),desitination(2),desitination(3));
% Set motor EPROM to position control mode
motorPos = [-q1+4.5*pi/180 q2+5*pi/180 q3+100*pi/180 q4-45*pi/180 55*pi/180]
motorPos/pi*180

[motorFB, eFB] = readFB(s, numID);
motorFB 

angle1 = linspace(motorFB(1),motorPos(1),50);
angle2 = linspace(motorFB(2),motorPos(2),50);
angle3 = linspace(motorFB(3),motorPos(3),50);
angle4 = linspace(motorFB(4),motorPos(4),50);

for i = 1:length(angle1)
    Pos = [angle1(i) angle2(i) angle3(i) angle4(i) 35*pi/180];
    sendJointPos(s, Pos, numID);
end

%% Task 1 Execution

catchHeigh = 2;

initial_A = 'egfbehedcdbcfddfcadgfggcf';
initial_B = [2 1 1 1 1 1 4 5 3 5 5 2 3 2 1 3 1 1 5 1 2 5 2 1 1];

final_A = 'efbcgfdadaccgdfdacahfhgde';
final_B = [4 3 5 3 1 1 5 0 5 0 6 3 5 4 3 5 0 1 0 1 4 3 3 1 1];

captureTimes = 0;

 %for i = 1:length(initial_A)
    i = 11;
     if final_B(i) == 0
        [x_initial y_initial] = convert(initial_A(i), initial_B(i));
        [x_final y_final] = convert(final_A(i), final_B(i));
        x_final = x_final + captureTimes * 3;
        initial = [x_initial y_initial catchHeigh];
        final = [x_final y_final catchHeigh];
        Q = task1(initial, final);
        captureTimes = captureTimes + 1;
     else
        [x_initial y_initial] = convert(initial_A(i), initial_B(i));
        [x_final y_final] = convert(final_A(i), final_B(i));
        initial = [x_initial y_initial catchHeigh];
        final = [x_final y_final catchHeigh];
        Q = task1(initial, final);
     end
    
    [a b] = size(Q);
    
    feedback = zeros(a,4);
    for j = 1:a
        [motorFB, eFB] = readFB(s, numID);
        feedback(j,:) = [motorFB(1) motorFB(2) motorFB(3) motorFB(4)];
        Q(j,1) = -Q(j,1) + 4.5*pi/180;
        Q(j,2) = Q(j,2) + 5/180*pi;
        Q(j,3) = Q(j,3) + 100/180*pi;
        Q(j,4) = Q(j,4) - 45/180*pi;
        motorPos = Q(j,:);
    
        sendJointPos(s, motorPos, numID);  

    end
 %end


 
%% Close Serial Connection - MUST DO PRIOR TO DISCONNECTING HARDWARE OR CLEARING VARIABLES

fclose(s);

%%
Q = [0 1.6766 -0.4262 -0.1159 0.7854];
Q = Q/pi*180
[x y z] = forward(0, 89, -122, 33)
%% Task 1 Function

function Q = task1(initial, final)
R = [17; 0; 10];


H = 8;
travelT = 3.5;
Ts = 0.02;
Q = [];
releaseAngle = 55*pi/180;
catchAngle = 10*pi/180;
% Movement 1: ready to departure
Via1 = [initial(1);initial(2);initial(3)+H];
% Segment 1, travel in xy plane
[ax bx cx dx] = cubicPolynomial(R(1), 0, Via1(1), 0, 0, travelT);
[ay by cy dy] = cubicPolynomial(R(2), 0, Via1(2), 0, 0, travelT);

t1 = 0:Ts:travelT;
x = coordinate(ax, bx, cx, dx, 0, travelT, Ts);
y = coordinate(ay, by, cy, dy, 0, travelT, Ts);
z = R(3) * ones(1,length(t1));

for i = 1:length(x)
    [q1 q2 q3 q4] = inverse(x(i),y(i),z(i));
    Q = [Q;[q1 q2 q3 q4 releaseAngle]];
end

% Segment 2, travel along z
[az bz cz dz] = cubicPolynomial(Via1(3), 0, initial(3), 0, travelT, 2*travelT);
t2 = travelT:Ts:2*travelT;
x = x(end) * ones(1,length(t2));
y = y(end) * ones(1,length(t2));
z = coordinate(az, bz, cz, dz, travelT, 2*travelT, Ts);

for i = 1:length(x)
    [q1 q2 q3 q4] = inverse(x(i),y(i),z(i));
    Q = [Q;[q1 q2 q3 q4 releaseAngle]];
end

%----------------------------------------------------------------------------------------------
% catch the chess


% Delay
for i = 1:50    
    Q = [Q;[q1 q2 q3 q4 releaseAngle]];
end

for q = releaseAngle:-1*pi/180:catchAngle
    Q = [Q;[q1 q2 q3 q4 q]];
end
%----------------------------------------------------------------------------------------------

% Movement 2: departure to destination
Via2 = [final(1);final(2);final(3)+H];

% Segment 1, travel along z
[az bz cz dz] = cubicPolynomial(initial(3), 0, Via1(3), 0, 0, travelT);
x = x(end) * ones(1,length(t1));
y = y(end) * ones(1,length(t1));
z = coordinate(az, bz, cz, dz, 0, travelT, Ts);

for i = 1:length(x)
    [q1 q2 q3 q4] = inverse(x(i),y(i),z(i));
    Q = [Q;[q1 q2 q3 q4 catchAngle]];
end

% Segment 2, travel in xy plane
[ax bx cx dx] = cubicPolynomial(Via1(1), 0, Via2(1), 0, travelT, 2*travelT);
[ay by cy dy] = cubicPolynomial(Via1(2), 0, Via2(2), 0, travelT, 2*travelT);
x = coordinate(ax, bx, cx, dx, travelT, 2*travelT, Ts);
y = coordinate(ay, by, cy, dy, travelT, 2*travelT, Ts);
z = z(end) * ones(1,length(t2));

for i = 1:length(x)
    [q1 q2 q3 q4] = inverse(x(i),y(i),z(i));
    Q = [Q;[q1 q2 q3 q4 catchAngle]];
end

% Segment 3, travel along z
[az bz cz dz] = cubicPolynomial(Via2(3), 0, final(3), 0, 2*travelT, 3*travelT);
t3 = 2*travelT:Ts:3*travelT;
x = x(end) * ones(1,length(t3));
y = y(end) * ones(1,length(t3));
z = coordinate(az, bz, cz, dz, 2*travelT, 3*travelT, Ts);

for i = 1:length(x)
    [q1 q2 q3 q4] = inverse(x(i),y(i),z(i));
    Q = [Q;[q1 q2 q3 q4 catchAngle]];
end

%----------------------------------------------------------------------------------------
% place the chess

% Delay
for i = 1:50
    Q = [Q;[q1 q2 q3 q4 catchAngle]];
end

for q = catchAngle:1*pi/180:releaseAngle
    Q = [Q;[q1 q2 q3 q4 q]];
end

%----------------------------------------------------------------------------------------

% Movement 3: destination to ready
% Segment 1, travel along z
[az bz cz dz] = cubicPolynomial(final(3), 0, Via2(3), 0, 0, travelT);

x = x(end) * ones(1, length(t1));
y = y(end) * ones(1, length(t1));
z = coordinate(az, bz, cz, dz, 0, travelT, Ts);

for i = 1:length(x)
    [q1 q2 q3 q4] = inverse(x(i),y(i),z(i));
    Q = [Q;[q1 q2 q3 q4 releaseAngle]];
end

% Segment 2, travel in xy plane
[ax bx cx dx] = cubicPolynomial(Via2(1), 0, R(1), 0, travelT, 2*travelT);
[ay by cy dy] = cubicPolynomial(Via2(2), 0, R(2), 0, travelT, 2*travelT);

x = coordinate(ax, bx, cx, dx, travelT, 2*travelT, Ts);
y = coordinate(ay, by, cy, dy, travelT, 2*travelT, Ts);
z = z(end) * ones(1,length(t2));

for i = 1:length(x)
    [q1 q2 q3 q4] = inverse(x(i),y(i),z(i));
    Q = [Q;[q1 q2 q3 q4 releaseAngle]];
end

end

%% A function to calculate cubic polynomial 

function [a0, a1, a2, a3] = cubicPolynomial(x0, x0_dot, xf, xf_dot, t0, tf)

% Cubic polynomial x(t)=a3t^3+a2t^2+a1t+a0  
A = [1 t0 t0^2 t0^3; 0 1 2*t0 3*t0^2; 1 tf tf^2 tf^3; 0 1 2*tf 3*tf^2];
X = [x0; x0_dot; xf; xf_dot];

b = A\X;
a0 = b(1);
a1 = b(2);
a2 = b(3);
a3 = b(4);
end

%% A function to calculate coordinate with given polynomial

function co = coordinate(a0, a1, a2, a3, t0, tf, step)
t = t0:step:tf;
co = zeros(1,length(t));

for i = 1:length(t)
    co(i) = a0 + a1*t(i) + a2*t(i)^2 + a3*t(i)^3;
end
end

%%  A function find the inverse kinematics

function [Q1 Q2 Q3 Q4] = inverse(x, y, z)
l1 = 18;
l2 = 18;
l3 = 20;
l4 = 15;

% Q1
Q1 = atand(y/x);
% Q3
Q3_nu = x^2 + y^2 + z^2 + l1^2 - l2^2 - l3^2 + l4^2 - 2*l1*l4 - 2*l1*z + 2*l4*z;
Q3_de = 2*l2*l3;
Q3 = acosd(Q3_nu/Q3_de);
if Q3 > 0
    Q3 = -Q3;
end
% Q2
alpha = atand((l1-l4-z)/(sqrt(x^2+y^2)));
L = sqrt(x^2 + y^2 + (l1 - l4 - z)^2);
Q2 = asind(-l3*sind(Q3)/L) - alpha;
% Q4
Q4 = -Q2-Q3;


Q1 = Q1/180*pi;
Q2 = Q2/180*pi;
Q3 = Q3/180*pi;
Q4 = Q4/180*pi;

end

%%  A function find the forward kinematics

function [x y z] = forward(Q1, Q2, Q3, Q4)

l1 = 18;
l2 = 18;
l3 = 20;
l4 = 15;
Q5 = 0;

% Find the coordinate of the end effector in frame 0
T01 = translationMatrix(0, 0, l1, Q1);
T12 = translationMatrix(0, 90, 0, Q2);
T23 = translationMatrix(l2, 0, 0, Q3);
T34 = translationMatrix(l3, 0, 0, Q4);
T45 = translationMatrix(0, -90, 0, Q5);
T5E = translationMatrix(0, 180, l4, 0);

T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
T0E = T04*T45*T5E;

% r2 = T02*[l2;0;0;1];
% r2 = r2(1:3,1);
% 
% r3 = T03*[l3;0;0;1];
% r3 = r3(1:3,1);
% 
% r4 = T04*[0;-l4;0;1];
% r4 = r4(1:3,1);

rE = T0E(1:3,end);
x = rE(1);
y = rE(2);
z = rE(3);

% x = [0 0 r2(1) r3(1) r4(1)];
% y = [0 0 r2(2) r3(2) r4(2)];
% z = [0 l1 r2(3) r3(3) r4(3)];
end

%% A function to find translation matrix

function T = translationMatrix(a, alpha, d, theta)
Rx = [1 0 0 0;0 cosd(alpha) -sind(alpha) 0; 0 sind(alpha) cosd(alpha) 0; 0 0 0 1];
Dx = [1 0 0 a; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Dz = [1 0 0 0; 0 1 0 0; 0 0 1 d; 0 0 0 1];
Rz = [cosd(theta) -sind(theta) 0 0; sind(theta) cosd(theta) 0 0;0 0 1 0; 0 0 0 1];
T = Rx*Dx*Dz*Rz;
end


%%  Given location find task space coordinate

function [x_final,y_final] = convert(x,y)

if y == 0
    x_final = 20;
    y_final = 16;
  
else
    X = ['a' 'b' 'c' 'd' 'e' 'f' 'g' 'h'];
    x = find(X==x);

    x_final = 17-3.5/2 + y*3.5;
    y_final = 1.75 - (x-4)*3.5;
end
end
%% Find Jacobian Matrix

function J = jacobian(Q1,Q2,Q3,Q4)

Q5 = 0;
Z1 = [0;0;1];
Z2 = [sin(Q1);-cos(Q1);0];
Z3 = [sin(Q1);-cos(Q1);0];
Z4 = [sin(Q1);-cos(Q1);0];
Z5 = [0;0;1];

% Find the translation matrix Tab (from frame b to a)            
T01 = translationMatrix(0, 0, l1, Q1);
T12 = translationMatrix(0, 90, 0, Q2);
T23 = translationMatrix(l2, 0, 0, Q3);
T34 = translationMatrix(l3, 0, 0, Q4);
T45 = translationMatrix(0, -90, 0, Q5);
T5E = translationMatrix(0, 180, l4, 0);

T02 = T01*T12;
T03 = T01*T12*T23;
T04 = T01*T12*T23*T34;
T05 = T01*T12*T23*T34*T45;

T0E = T01*T12*T23*T34*T45*T5E;

r1E = T0E(1:3,4) - T01(1:3,4);
r2E = T0E(1:3,4) - T02(1:3,4);
r3E = T0E(1:3,4) - T03(1:3,4);
r4E = T0E(1:3,4) - T04(1:3,4);
r5E = T0E(1:3,4) - T05(1:3,4);

% Find the Jacobian matrix for the end effector
J = [cross(Z1,r1E) cross(Z2,r2E) cross(Z3,r3E) cross(Z4,r4E) cross(Z5,r5E); ...
    Z1 Z2 Z3 Z4 Z5];
end
%% Find Rotated Jacobian Matrix

function J_R = jacobianr(Q1,Q2,Q3,Q4)

Q5 = 0;
Z1 = rotz(45)*[0;0;1];
Z2 = rotz(45)*[sin(Q1);-cos(Q1);0];
Z3 = rotz(45)*[sin(Q1);-cos(Q1);0];
Z4 = rotz(45)*[sin(Q1);-cos(Q1);0];
Z5 = rotz(45)*[0;0;1];

% Find the translation matrix Tab (from frame b to a)            
Tr0 = [rotz(45) [0;0;0];0 0 0 1];
T01 = translationMatrix(0, 0, l1, Q1);
T12 = translationMatrix(0, 90, 0, Q2);
T23 = translationMatrix(l2, 0, 0, Q3);
T34 = translationMatrix(l3, 0, 0, Q4);
T45 = translationMatrix(0, -90, 0, Q5);
T5E = translationMatrix(0, 180, l4, 0);
T01 = Tr0*T01;

T02 = T01*T12;
T03 = T01*T12*T23;
T04 = T01*T12*T23*T34;
T05 = T01*T12*T23*T34*T45;

T0E = T01*T12*T23*T34*T45*T5E;

r1E = T0E(1:3,4) - T01(1:3,4);
r2E = T0E(1:3,4) - T02(1:3,4);
r3E = T0E(1:3,4) - T03(1:3,4);
r4E = T0E(1:3,4) - T04(1:3,4);
r5E = T0E(1:3,4) - T05(1:3,4);

% Find the Jacobian matrix for the end effector
J = [cross(Z1,rotz(45)*r1E) cross(Z2,rotz(45)*r2E) ...
    cross(Z3,rotz(45)*r3E) cross(Z4,rotz(45)*r4E) ...
    cross(Z5,rotz(45)*r5E)]
end
%%