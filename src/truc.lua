BoatState.msg :
geometry_msgs/Vector3 position
std_msgs/Float64 heading
std_msgs/Float64 lin_spd

MotorsCmd.msg :
std_msgs/Float64 u_left
std_msgs/Float64 u_right

Tick.msg :
std_msgs/Int32 nL
std_msgs/Float64 dt

function subscriber_boat_state_callback(msg)
   -- This is the subscriber callback function when receiving /boat_state  topic
   hd_obj = 0
   spd_obj = 50
   kp = 0.6 -- to adjust
   ke = 0.35 -- to adjust
   x = msg["position"]["x"]
   y = msg["position"]["y"]
   z = msg["position"]["z"]
   hd = msg["heading"]["data"]
   spd = msg["lin_spd"]["data"]
   e = ke*(hd-hd_obj)
   u_left = 0.5*spd_obj*(1+kp*e)
   u_right = 0.5*spd_obj*(1-kp*e)
   sim.setJointTargetVelocity(leftMotor,u_left)
   sim.setJointTargetVelocity(rightMotor,u_right)

