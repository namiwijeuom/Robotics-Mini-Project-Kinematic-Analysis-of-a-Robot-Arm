%% Use the Denavit-Hartenberg (DH) parameters of the Puma560® manipulator robot 
% to incrementally build a rigid body tree robot model. 
% Specify the relative DH parameters for each joint as you attach them. 
% Visualize the robot frames, and interact with the final model.
clear all;
close all;

dhparams = [0   	pi/2	0   	0;
            0.4318	0       0       0
            0.0203	-pi/2	0.15005	0;
            0   	pi/2	0.4318	0;
            0       -pi/2	0   	0;
            0       0       0       0];
        
robot = rigidBodyTree;
%disp(robot);

bodies = cell(6,1);
joints = cell(6,1);
for i = 1:6
    bodies{i} = rigidBody(['body' num2str(i)]);
    joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
    setFixedTransform(joints{i},dhparams(i,:),"dh");
    
    bodies{i}.Joint = joints{i};
    if i == 1 % Add first body to base
        addBody(robot,bodies{i},"base")
    else % Add current body to previous body by name
        addBody(robot,bodies{i},bodies{i-1}.Name)
    end
end

showdetails(robot)

figure(Name="PUMA Robot Model")
show(robot);

%% Interact with Robot Model
figure(Name="Interactive GUI")
gui = interactiveRigidBodyTree(robot,MarkerScaleFactor=0.5);
