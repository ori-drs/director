% demo of an ik trajectory for hyq
% robot moves base link forward and down
% trajectory published to LCM

if (1==1)

disp(exist('ikServerStarted'))

%-------- startup --------

format long e
addpath_control
addpath([getenv('DRC_BASE'), '/software/director/src/matlab'])
robotURDF = [getenv('DRC_BASE'), '/software/models/hyq_description/robots/hyq.urdf'];
fixed_point_file = [getenv('DRC_BASE'), '/software/models/hyq_description/hyq_fp.mat'];
left_foot_link = 'lh_foot';
right_foot_link = 'rh_foot';
pelvis_link = 'trunk';
runIKServer
end

%------ startup end ------

disp(exist('ikServerStarted'))
reach_start = [0.0;0.0;0.627;0.0;0.0;0.0;-0.139349267;0.664807618;-1.33601511;-0.158358365;0.679332614;-1.34880483;-0.176648319;-0.681990683;1.35311472;-0.147684306;-0.695933938;1.33601511];

%-------- runIk --------

excluded_collision_groups = struct('name',{},'tspan',{});

default_shrink_factor = 0.9;
qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(default_shrink_factor);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
pts = [0;0;0];
qsc_constraint_0 = qsc_constraint_0.addContact(links.lh_foot, pts);
qsc_constraint_0 = qsc_constraint_0.addContact(links.rh_foot, pts);
qsc_constraint_0 = qsc_constraint_0.addContact(links.lf_foot, pts);
qsc_constraint_0 = qsc_constraint_0.addContact(links.rf_foot, pts);


point_in_link_frame = [0.341; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.lh_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_1 = WorldPositionConstraint(r, links.lh_foot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);


point_in_link_frame = [0.341; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.rh_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_2 = WorldPositionConstraint(r, links.rh_foot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);


point_in_link_frame = [0.341; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.lf_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_3 = WorldPositionConstraint(r, links.lf_foot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);


point_in_link_frame = [0.341; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.rf_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_4 = WorldPositionConstraint(r, links.rf_foot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);


point_in_link_frame = [0.0; 0.0; 0.0];
ref_frame = [1.0, 0.0, 0.0, 0.11861187085311649; 0.0, 1.0, 0.0, 4.4408920985006262e-16; 0.0, 0.0, 1.0, 0.58463528107110574; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0001; -0.0001; -0.0001];
upper_bounds = [0.0; 0.0; 0.0] + [0.0001; 0.0001; 0.0001];
position_constraint_5 = WorldPositionInFrameConstraint(r, links.trunk, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);


quat_constraint_6 = WorldQuatConstraint(r, links.trunk, [1.0; 0.0; 0.0; 0.0], 0.0, [1.0, 1.0]);


q_nom = [q_nom; zeros(r.getNumPositions()-numel(q_nom),1)];
reach_start = [reach_start; zeros(r.getNumPositions()-numel(reach_start),1)];
active_constraints = {qsc_constraint_0, position_constraint_1, position_constraint_2, position_constraint_3, position_constraint_4, position_constraint_5, quat_constraint_6};
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
% visualise the pose found by ik
%send_est_pose_hyq(q_end)
%keyboard

disp(q_end)
disp(info)
q_end = [0.1185118708531165;0.000100000000000011;0.5847352810711057;4.982541058788652e-05;-4.270415227041483e-06;7.9759507324112e-05;-0.1493469330292333;0.9566386304005728;-1.457730911706156;-0.170287741875284;0.9694948358249967;-1.464177732567986;-0.1894968373740168;-0.5128984446395547;1.484390968205387;-0.158703322059978;-0.536149506386811;1.48179577040049];
environment_urdf_string = [];
s = s.setEnvironment(environment_urdf_string);
r = s.robot_and_environment;
reach_start = [0.0;0.0;0.627;0.0;0.0;0.0;-0.139349267;0.664807618;-1.33601511;-0.158358365;0.679332614;-1.34880483;-0.176648319;-0.681990683;1.35311472;-0.147684306;-0.695933938;1.33601511];

%-------- runIk --------

excluded_collision_groups = struct('name',{},'tspan',{});

default_shrink_factor = 0.9;
qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(default_shrink_factor);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
pts = [0;0;0];
qsc_constraint_0 = qsc_constraint_0.addContact(links.lh_foot, pts);
qsc_constraint_0 = qsc_constraint_0.addContact(links.rh_foot, pts);
qsc_constraint_0 = qsc_constraint_0.addContact(links.lf_foot, pts);
qsc_constraint_0 = qsc_constraint_0.addContact(links.rf_foot, pts);


point_in_link_frame = [0.341; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.lh_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_1 = WorldPositionConstraint(r, links.lh_foot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);


point_in_link_frame = [0.341; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.rh_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_2 = WorldPositionConstraint(r, links.rh_foot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);


point_in_link_frame = [0.341; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.lf_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_3 = WorldPositionConstraint(r, links.lf_foot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);


point_in_link_frame = [0.341; 0; 0];
kinsol = r.doKinematics(reach_start);
xyz_quat = r.forwardKin(kinsol, links.rf_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [-0.0001; -0.0001; -0.0001];
upper_bounds = xyz_quat(1:3) + [0.0001; 0.0001; 0.0001];
position_constraint_4 = WorldPositionConstraint(r, links.rf_foot, point_in_link_frame, lower_bounds, upper_bounds, [0.0, 1.0]);


point_in_link_frame = [0.0; 0.0; 0.0];
ref_frame = [1.0, 0.0, 0.0, 0.11861187085311649; 0.0, 1.0, 0.0, 4.4408920985006262e-16; 0.0, 0.0, 1.0, 0.58463528107110574; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [-0.0001; -0.0001; -0.0001];
upper_bounds = [0.0; 0.0; 0.0] + [0.0001; 0.0001; 0.0001];
position_constraint_5 = WorldPositionInFrameConstraint(r, links.trunk, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);


quat_constraint_6 = WorldQuatConstraint(r, links.trunk, [1.0; 0.0; 0.0; 0.0], 0.0, [1.0, 1.0]);


q_nom = [q_nom; zeros(r.getNumPositions()-numel(q_nom),1)];
reach_start = [reach_start; zeros(r.getNumPositions()-numel(reach_start),1)];
active_constraints = {qsc_constraint_0, position_constraint_1, position_constraint_2, position_constraint_3, position_constraint_4, position_constraint_5, quat_constraint_6};
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
q_end = [0.1185118708531165;0.000100000000000011;0.5847352810711057;4.982541058788652e-05;-4.270415227041483e-06;7.9759507324112e-05;-0.1493469330292333;0.9566386304005728;-1.457730911706156;-0.170287741875284;0.9694948358249967;-1.464177732567986;-0.1894968373740168;-0.5128984446395547;1.484390968205387;-0.158703322059978;-0.536149506386811;1.48179577040049];
reach_end = [0.1185118708531165;0.000100000000000011;0.5847352810711057;4.982541058788652e-05;-4.270415227041483e-06;7.9759507324112e-05;-0.1493469330292333;0.9566386304005728;-1.457730911706156;-0.170287741875284;0.9694948358249967;-1.464177732567986;-0.1894968373740168;-0.5128984446395547;1.484390968205387;-0.158703322059978;-0.536149506386811;1.48179577040049];
reach_start = [0.0;0.0;0.627;0.0;0.0;0.0;-0.139349267;0.664807618;-1.33601511;-0.158358365;0.679332614;-1.34880483;-0.176648319;-0.681990683;1.35311472;-0.147684306;-0.695933938;1.33601511];
reach_end = [0.1185118708531165;0.000100000000000011;0.5847352810711057;4.982541058788652e-05;-4.270415227041483e-06;7.9759507324112e-05;-0.1493469330292333;0.9566386304005728;-1.457730911706156;-0.170287741875284;0.9694948358249967;-1.464177732567986;-0.1894968373740168;-0.5128984446395547;1.484390968205387;-0.158703322059978;-0.536149506386811;1.48179577040049];

%-------- runIkTraj --------

reach_start = [reach_start; zeros(r.getNumPositions()-numel(reach_start),1)];
reach_end = [reach_end; zeros(r.getNumPositions()-numel(reach_end),1)];
reach_start = [reach_start; zeros(r.getNumPositions()-numel(reach_start),1)];
excluded_collision_groups = struct('name',{},'tspan',{});

end_effector_name = 'none';
end_effector_name_left = 'none';
end_effector_pt = [];
default_shrink_factor = 0.9;
posture_constraint_0 = PostureConstraint(r, [1.0, 1.0]);
joint_inds = [joints.base_x; joints.base_y; joints.base_z; joints.base_roll; joints.base_pitch; joints.base_yaw; joints.lf_haa_joint; joints.lf_hfe_joint; joints.lf_kfe_joint; joints.rf_haa_joint; joints.rf_hfe_joint; joints.rf_kfe_joint; joints.lh_haa_joint; joints.lh_hfe_joint; joints.lh_kfe_joint; joints.rh_haa_joint; joints.rh_hfe_joint; joints.rh_kfe_joint];
joints_lower_limit = reach_end(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = reach_end(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_0 = posture_constraint_0.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);


point_in_link_frame = [0.341; 0; 0];
kinsol = r.doKinematics(reach_end);
xyz_quat = r.forwardKin(kinsol, links.lh_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [0.0; 0.0; 0.0];
upper_bounds = xyz_quat(1:3) + [0.0; 0.0; 0.0];
position_constraint_1 = WorldPositionConstraint(r, links.lh_foot, point_in_link_frame, lower_bounds, upper_bounds, [-inf, inf]);


point_in_link_frame = [0.341; 0; 0];
kinsol = r.doKinematics(reach_end);
xyz_quat = r.forwardKin(kinsol, links.rh_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [0.0; 0.0; 0.0];
upper_bounds = xyz_quat(1:3) + [0.0; 0.0; 0.0];
position_constraint_2 = WorldPositionConstraint(r, links.rh_foot, point_in_link_frame, lower_bounds, upper_bounds, [-inf, inf]);


point_in_link_frame = [0.341; 0; 0];
kinsol = r.doKinematics(reach_end);
xyz_quat = r.forwardKin(kinsol, links.lf_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [0.0; 0.0; 0.0];
upper_bounds = xyz_quat(1:3) + [0.0; 0.0; 0.0];
position_constraint_3 = WorldPositionConstraint(r, links.lf_foot, point_in_link_frame, lower_bounds, upper_bounds, [-inf, inf]);


point_in_link_frame = [0.341; 0; 0];
kinsol = r.doKinematics(reach_end);
xyz_quat = r.forwardKin(kinsol, links.rf_foot, point_in_link_frame, 2);
lower_bounds = xyz_quat(1:3) + [0.0; 0.0; 0.0];
upper_bounds = xyz_quat(1:3) + [0.0; 0.0; 0.0];
position_constraint_4 = WorldPositionConstraint(r, links.rf_foot, point_in_link_frame, lower_bounds, upper_bounds, [-inf, inf]);


active_constraints = {posture_constraint_0, position_constraint_1, position_constraint_2, position_constraint_3, position_constraint_4};
t = [0.0, 1.0];
nt = size(t, 2);
clear xtraj;
clear info;
clear infeasible_constraint;
additionalTimeSamples = [];
options = struct();
options.MajorIterationsLimit = 500;
options.MajorFeasibilityTolerance = 1e-06;
options.MajorOptimalityTolerance = 0.0001;
options.FixInitialState = true;
s = s.setupOptions(options);
ikoptions = s.ikoptions.setAdditionaltSamples(additionalTimeSamples);


q_nom_traj = PPTrajectory(foh(t, repmat(reach_start, 1, nt)));
q_seed_traj = PPTrajectory(spline([t(1), t(end)], [zeros(r.getNumPositions(),1), reach_start, reach_end, zeros(r.getNumPositions(),1)]));


[xtraj, info, infeasible_constraint] = inverseKinTraj(r, t, q_seed_traj, q_nom_traj, active_constraints{:}, ikoptions);


if (info > 10) display(infeasibleConstraintMsg(infeasible_constraint)); end;
if ~isempty(xtraj), qtraj = xtraj(1:r.getNumPositions()); else, qtraj = []; end;
if ~isempty(qtraj), qtraj_orig = qtraj; end;
if ~isempty(qtraj), joint_v_max = repmat(30.0*pi/180, r.getNumVelocities()-6, 1); end;
if ~isempty(qtraj), xyz_v_max = repmat(0.05, 3, 1); end;
if ~isempty(qtraj), rpy_v_max = repmat(2*pi/180, 3, 1); end;
if ~isempty(qtraj), v_max = [xyz_v_max; rpy_v_max; joint_v_max]; end;
if ~isempty(qtraj), v_max(r.findPositionIndices('back')) = 10.0*pi/180; end;
max_body_translation_speed = 0.5;
max_body_rotation_speed = 10;
rescale_body_ids = [];
rescale_body_pts = reshape([], 3, []);
body_rescale_options = struct('body_id',rescale_body_ids,'pts',rescale_body_pts,'max_v',max_body_translation_speed,'max_theta',max_body_rotation_speed,'robot',r);

%--- pointwise ik --------

if ~isempty(qtraj), num_pointwise_time_points = 20; end;
if ~isempty(qtraj), pointwise_time_points = linspace(qtraj.tspan(1), qtraj.tspan(2), num_pointwise_time_points); end;
if ~isempty(qtraj), q_seed_pointwise = qtraj.eval(pointwise_time_points); end;
if ~isempty(qtraj), q_seed_pointwise = q_seed_pointwise(1:r.getNumPositions(),:); end;
if ~isempty(qtraj), [qtraj_pw, info_pw] = inverseKinPointwise(r, pointwise_time_points, q_seed_pointwise, q_seed_pointwise, active_constraints{:}, ikoptions); else, qtraj_pw = []; end;
if ~isempty(qtraj_pw), qtraj_pw = PPTrajectory(foh(pointwise_time_points, qtraj_pw)); end;
if ~isempty(qtraj_pw), info = info_pw(end); end;
if ~isempty(qtraj_pw), if (any(info_pw > 10)) disp('pointwise info:'); disp(info_pw); end; end;
if ~isempty(qtraj_pw), qtraj_orig = qtraj_pw; end;

%--- pointwise ik end --------

if ~isempty(qtraj_orig), qtraj = rescalePlanTiming(qtraj_orig, v_max, 2, 0.3, body_rescale_options); end;
if ~isempty(qtraj_orig), s.publishTraj(qtraj, info); end;

%--- runIKTraj end --------

disp(info)
