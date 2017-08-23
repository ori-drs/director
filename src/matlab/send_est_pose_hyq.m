function send_est_pose_hyq(q_end)
joint_name = {'lf_haa_joint', 'lf_hfe_joint', 'lf_kfe_joint',...
               'rf_haa_joint', 'rf_hfe_joint', 'rf_kfe_joint',...
               'lh_haa_joint', 'lh_hfe_joint', 'lh_kfe_joint',...
               'rh_haa_joint', 'rh_hfe_joint', 'rh_kfe_joint'};
joint_pos = q_end(7:end)';
pos = q_end(1:3);
quat = rpy2quat(q_end(4:6));

%if (info ==1)
send_est_pose(pos,quat,joint_name, joint_pos)
%else
%  disp('fail')
%end 
