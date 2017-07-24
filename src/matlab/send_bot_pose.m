function send_bot_pose(pos,quat, varargin)
if nargin>2
    channel=varargin{1};
else
    channel='POSE_BODY_ALT';
end

status = bot_core.pose_t();
status.utime  = etime(clock,[1970 1 1 0 0 0])*1000000;
status.pos = pos;
status.orientation = quat;
lc = lcm.lcm.LCM.getSingleton();
lc.publish(channel, status);
%pause(0.25)
