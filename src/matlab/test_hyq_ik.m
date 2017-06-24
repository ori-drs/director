% simple hyq ik problem
% single foot position constraint
% and posture constraint

if (1==0)
disp(exist('ikServerStarted'))

%-------- startup --------

format long e
addpath_control
addpath([getenv('DRC_BASE'), '/software/director/src/matlab'])
robotURDF = [getenv('DRC_BASE'), '/software/models/hyq_description/robots/hyq.urdf'];
fixed_point_file = [getenv('DRC_BASE'), '/software/models/hyq_description/hyq_fp.mat'];
left_foot_link = 'lf_foot';
right_foot_link = 'lh_foot';
pelvis_link = 'trunk';
runIKServer
end

%------ startup end ------

disp(exist('ikServerStarted'))
reach_start = [0.0;0.0;0.627;0.0;0.0;0.0;-0.139349267;0.664807618;-1.33601511;-0.158358365;0.679332614;-1.34880483;-0.176648319;-0.681990683;1.35311472;-0.147684306;-0.695933938;1.33601511];

%-------- runIk --------

excluded_collision_groups = struct('name',{},'tspan',{});

default_shrink_factor = 0.2;
qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(default_shrink_factor);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
qsc_constraint_0 = qsc_constraint_0.addContact(links.lf_foot, l_foot_pts);
qsc_constraint_0 = qsc_constraint_0.addContact(links.lh_foot, r_foot_pts);

keyboard

point_in_link_frame = [0.341; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.lf_foot, point_in_link_frame, 2);
% temp
xyz_quat(1)=0.24
xyz_quat(2)=0.30
xyz_quat(3)=0.15
%xyz_quat2 = xyz_quat
lower_bounds = xyz_quat(1:3) + [-0.001; -0.001; -0.001];
upper_bounds = xyz_quat(1:3) + [0.001; 0.001; 0.001];
position_constraint_1 = WorldPositionConstraint(r, links.lf_foot, point_in_link_frame, lower_bounds, upper_bounds, [1.0, 1.0]);quaternion_constraint_1 = WorldQuatConstraint(r, links.lf_foot, xyz_quat(4:7), 0.0017453292519943296, [1.0, 1.0]);

send_bot_pose(xyz_quat(1:3), xyz_quat(4:7) );

point_in_link_frame = [0; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.lh_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_2 = WorldPositionConstraint(r, links.lh_foot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);quaternion_constraint_2 = WorldQuatConstraint(r, links.lh_foot, xyz_quat(4:7), 0.0017453292519943296, [0.0, 1.0]);


posture_constraint_3 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_x; joints.base_y; joints.base_z; joints.base_roll; joints.base_pitch; joints.base_yaw];
tmp = reach_start(3);
reach_start(3) = 0.6;
joints_lower_limit = reach_start(joint_inds) + [-0.1; -0.1; -0.1; 0.0; 0.0; 0.0];
joints_upper_limit = reach_start(joint_inds) + [0.1; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_3 = posture_constraint_3.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);
reach_start(3) = tmp;


point_in_link_frame = [0.0; 0.0; 0.0];
ref_frame = [-0.87389479432356665, 0.47705381193219115, -0.093421351816042206, 0.081152687941618451; 0.22315985882824019, 0.22296658070352091, -0.94893918735456384, 0.39430916747347905; -0.43186521724020688, -0.85012091164160042, -0.30130859550919403, 0.7174225492070806; 0.0, 0.0, 0.0, 1.0];
ref_frame(1,4) = 0;
ref_frame(2,4) = 0;
ref_frame(3,4) = 0;
%ref_frame = [0,0,0,0;0,0,0,0;0,0,0,0; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0; -0.0; -0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
position_constraint_5 = WorldPositionInFrameConstraint(r, links.rh_foot, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);
%quat_constraint_6 = WorldQuatConstraint(r, links.rh_foot, [0.62478205122619512; -0.25087190468628373; 0.47024533896713572; -0.57059617689250941], 0.0, [1.0, 1.0]);

quat_constraint_6 = WorldQuatConstraint(r, links.rh_foot, [0.0; 0.0; 0.47024533896713572; -0.57059617689250941], 0.0, [1.0, 1.0]);




q_nom = [q_nom; zeros(r.getNumPositions()-numel(q_nom),1)];
reach_start = [reach_start; zeros(r.getNumPositions()-numel(reach_start),1)];
%active_constraints = {qsc_constraint_0, position_constraint_1, quaternion_constraint_1, position_constraint_2, quaternion_constraint_2, posture_constraint_3};
%active_constraints = {position_constraint_1, quaternion_constraint_1, position_constraint_2, quaternion_constraint_2, posture_constraint_3};
active_constraints = {position_constraint_1,  posture_constraint_3};
%active_constraints = {position_constraint_5, quat_constraint_6, posture_constraint_3};
%active_constraints = {posture_constraint_3};
ik_seed_pose = reach_start;
ik_nominal_pose = q_nom;
ik_seed_pose = [ik_seed_pose; zeros(r.getNumPositions()-numel(ik_seed_pose),1)];
ik_nominal_pose = [ik_nominal_pose; zeros(r.getNumPositions()-numel(ik_nominal_pose),1)];
options = struct();
options.MajorIterationsLimit = 500;
options.MajorFeasibilityTolerance = 1e-06;
options.MajorOptimalityTolerance = 0.0001;
options.MinDistance = 0.030000;
s = s.setupOptions(options);
clear q_end;
clear info;
clear infeasible_constraint;


use_collision = false;
[q_end, info, infeasible_constraint] = s.runIk(ik_seed_pose, ik_nominal_pose, active_constraints, use_collision);


q_end(s.robot.getNumPositions()+1:end) = [];

%-------- runIk end --------

disp(q_end)
disp(info)
infeasible_constraint
%q_end = [0.0;0.0;0.627;0.0;0.0;0.0;0.436332312999;-0.872664625997;-0.349065850399;0.436332312999;-0.872664625997;-0.349065850399;0.436332312999;0.872664625997;0.349065850399;0.436332312999;0.872664625997;0.349065850399];

joint_name = {'lf_haa_joint', 'lf_hfe_joint', 'lf_kfe_joint',...
               'rf_haa_joint', 'rf_hfe_joint', 'rf_kfe_joint',...
               'lh_haa_joint', 'lh_hfe_joint', 'lh_kfe_joint',...
               'rh_haa_joint', 'rh_hfe_joint', 'rh_kfe_joint'};
joint_pos = q_end(7:end)';
pos = q_end(1:3);
quat = rpy2quat(q_end(4:6));

if (info ==1)
  send_est_pose(pos,quat,joint_name, joint_pos)
else
  disp('fail')
end