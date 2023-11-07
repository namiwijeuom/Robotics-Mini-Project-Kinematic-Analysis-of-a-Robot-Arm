robot = rigidBodyTree('DataFormat','column','MaxNumBodies',3);

%% Specify arm lengths for the robot arm.
L1 = 0.3;
L2 = 0.3;

%% Add 'link1' body with 'joint1' joint.

body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');

%% Add 'link2' body with 'joint2' joint.
body = rigidBody('link2');
joint = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint, trvec2tform([L1,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link1');

%% Add 'tool' end effector with 'fix1' fixed joint.
body = rigidBody('tool');
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform([L2, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link2');

%% Robot Details_
showdetails(robot)
show(robot)

%% Define The Trajectory
% Define a circle to be traced over the course of 10 seconds. 
% This circle is in the xy plane with a radius of 0.15.

t = (0:0.2:10)'; % Time
count = length(t);
center = [0.3 0.1 0];
radius = 0.15;
theta = t*(2*pi/t(end));
points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];

%% Inverse Kinematics Solution

% Pre-allocate configuration solutions as a matrix qs.
q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count, ndof);

% Because the xy Cartesian points are the only important factors of 
% the end-effector pose for this workflow, specify a non-zero weight 
% for the fourth and fifth elements of the weight vector. 
% All other elements are set to zero.
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'tool';

qInitial = q0; % Use home configuration as the initial guess
for i = 1:count
    % Solve for the configuration satisfying the desired end effector position
    point = points(i,:);
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    
    % Store the configuration
    qs(i,:) = qSol;
    
    % Start from prior solution
    qInitial = qSol;
end

%% Animate The Solution
figure
show(robot,qs(1,:)');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(:,1),points(:,2),'k')
axis([-0.1 0.7 -0.3 0.5])

framesPerSecond = 15;
r = rateControl(framesPerSecond);
for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end
