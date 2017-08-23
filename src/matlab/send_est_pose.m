function send_est_pose(pos, quat, joint_name, joint_position)
% pos = [0,0,0.627];
% quat =[1,0,0,0]
% joint_name = {'lf_haa_joint', 'lf_hfe_joint', 'lf_kfe_joint',...
%               'rf_haa_joint', 'rf_hfe_joint', 'rf_kfe_joint',...
%               'lh_haa_joint', 'lh_hfe_joint', 'lh_kfe_joint',...
%               'rh_haa_joint', 'rh_hfe_joint', 'rh_kfe_joint'};
% joint_position = [-0.139, 0.665, -1.336, -0.158,...
%         0.68, -1.35, -0.17, -0.68,  1.35,...
%        -0.14, -0.69,  1.34];

status = bot_core.robot_state_t();
status.utime  = etime(clock,[1970 1 1 0 0 0])*1000000;
status.joint_name = joint_name;%{'aa','bb'};
status.joint_velocity = zeros(size(joint_position));% [0,0];
status.joint_position  = joint_position;
status.joint_effort = zeros(size(joint_position));
status.num_joints =size(joint_position,2);

pose = bot_core.position_3d_t();
pose.translation = bot_core.vector_3d_t();
pose.translation.x = pos(1);
pose.translation.y = pos(2);
pose.translation.z = pos(3);
pose.rotation = bot_core.quaternion_t();
pose.rotation.w = quat(1);
pose.rotation.x = quat(2);
pose.rotation.y = quat(3);
pose.rotation.z = quat(4);
status.pose = pose;


twist = bot_core.twist_t();
twist.linear_velocity = bot_core.vector_3d_t();
twist.angular_velocity = bot_core.vector_3d_t();
status.twist = twist;

ft = bot_core.force_torque_t();
status.force_torque = ft;

lc = lcm.lcm.LCM.getSingleton();
lc.publish('EST_ROBOT_STATE', status);
%pause(0.25)
