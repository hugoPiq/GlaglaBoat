require ("math")

function subscriber_cmd_vel_callback(msg)
   -- This is the subscriber callback function when receiving /cmd_vel  topic
   -- The msg is a Lua table defining linear and angular velocities
   -- spdLin = msg["linear"]["x"]*10.0
   -- spdAng = msg["angular"]["z"]*10.0
   -- kLin = -0.5
   -- kAng = -0.2
   -- spdLeft = kLin*spdLin+kAng*spdAng
   -- spdRight = kLin*spdLin-kAng*spdAng
   -- sim.setJointTargetVelocity(leftFrontMotor,spdLeft)
   -- sim.setJointTargetVelocity(rightFrontMotor,spdRight)
   -- sim.setJointTargetVelocity(leftRearMotor,spdLeft)
   -- sim.setJointTargetVelocity(rightRearMotor,spdRight)
   -- sim.addStatusbarMessage('cmd_vel subscriber receiver : spdLin ='..spdLin..',spdAng='..spdAng.." command : spdLeft="..spdLeft..",act="..spdRight)
   spdLeft = msg["data"][1]*10.0
   spdRight = msg["data"][2]*10.0
   
   objectHandle=sim.getObjectHandle("DynamiqueBoat")
   sim.addForceAndTorque(objectHandle, {spdLeft+spdRight, 0, 0}, {0, 0, spdLeft-spdRight})
   
   --sim.setJointTargetVelocity(rightMotor,spdRight)

end

function subscriber_cmd_motor_callback(msg)
   spdLeft = msg[0]
   spdRight = msg[1]
   print(spdLeft, spdRight)
end

function getGPS(objectName)
   objectHandle=sim.getObjectHandle(objectName)
   relTo=-1
   p=sim.getObjectPosition(objectHandle,relTo)
   rho = 6371000
   lx_ref = 48.199180
   ly_ref = -3.015480
   ly = p[2]/rho + ly_ref
   lx = p[1]/(rho*math.cos(ly)) + lx_ref
   lz = rho + p[3]
   return {
      latitude=ly, longitude=lx, altitude=lz,
      position_covariance={0, 0, 0, 0, 0, 0, 0, 0, 0}
   }
end

function getMagField(objectHandle,objectName,referenceHandle,referenceName)
   -- champ magnétique sur z (vers le haut) dans le frame World : -4600 µT
   B = -4600
   objectHandle=sim.getObjectHandle(objectName)
   relTo=-1
   o=sim.getObjectQuaternion(objectHandle,relTo)
   tf = getTransformStamped(objectHandle,objectName,referenceHandle,referenceName)
   -- get the quaternion to change from frame World to frame DD_Boat3D
   q0 = tf["transform"]["rotation"]["x"]
   q1 = tf["transform"]["rotation"]["y"]
   q2 = tf["transform"]["rotation"]["z"]
   q3 = tf["transform"]["rotation"]["w"]
   print("bonjour")
   --third row of the rotation matrix
   r20 = 2 * (q1 * q3 - q0 * q2)
   r21 = 2 * (q2 * q3 + q0 * q1)
   r22 = 2 * (q0 * q0 + q3 * q3) - 1
   -- /!\ Conversion orientation vers donnees boussole
   return {
      magnetic_field={x=B*r20, y=B*r21, z=B*r22},
      magnetic_field_covariance={0, 0, 0, 0, 0, 0, 0, 0, 0}
   }
end

function getImu(objectName)
   objectHandle=sim.getObjectHandle(objectName)
   relTo=-1
   o=sim.getObjectQuaternion(objectHandle,relTo)
   linVel,angVel=sim.getVelocity(objectHandle)
   
   return {
      orientation={x=o[1],y=o[2],z=o[3],w=o[4]},
      orientation_covariance={0,0,0,0,0,0,0,0,0},
      angular_velocity={x=angVel[1],y=angVel[2],z=angVel[3]},
      angular_velocity_covariance={0,0,0,0,0,0,0,0,0},
      linear_acceleration={x=linVel[1],y=linVel[2],z=linVel[3]},
      linear_acceleration_covariance={0,0,0,0,0,0,0,0,0}
   }
end
   

function getTransformStamped(objHandle,name,relTo,relToName)
   -- This function retrieves the stamped transform for a specific object
   t=sim.getSystemTime()
   p=sim.getObjectPosition(objHandle,relTo)
   o=sim.getObjectQuaternion(objHandle,relTo)
   return {
      header={
	 stamp=t,
	 frame_id=relToName
      },
      child_frame_id=name,
      transform={
	 translation={x=p[1],y=p[2],z=p[3]},
	 rotation={x=o[1],y=o[2],z=o[3],w=o[4]}
      }
   }
end

function sysCall_init()
   -- The child script initialization
   print ("init")
   objectName="DDBoat_3D"
   objectHandle=sim.getObjectHandle(objectName)
   referenceName="Plane"
   referenceHandle=sim.getObjectHandle(referenceName)
   -- get left and right motors handles
   --leftFrontMotor = sim.getObjectHandle("MotorFrontLeft")
   --rightFrontMotor = sim.getObjectHandle("MotorFrontRight")
   --leftRearMotor = sim.getObjectHandle("MotorRearLeft")
   --rightRearMotor = sim.getObjectHandle("MotorRearRight")
   --centralAxis = sim.getObjectHandle("CentralAxis")
   rosInterfacePresent=simROS
   -- Prepare the publishers and subscribers :
   if rosInterfacePresent then
      GPS=simROS.advertise('/Position','sensor_msgs/NavSatFix')
      Camera=simROS.advertise('/Image', 'sensor_msgs/Image')
      Compass=simROS.advertise('/MagField', 'sensor_msgs/MagneticField')
      IMU=simROS.advertise('/GyroAccelero', 'sensor_msgs/Imu')
      Motors=simROS.subscribe('/cmd_vel','std_msgs/Float64MultiArray','subscriber_cmd_vel_callback')
      --EncoderLeft=simROS.advertise('/RotSpeedLeft', 'std_msgs/Float64')
      --EncoderRight=simROS.advertise('/RotSpeedRight', 'std_msgs/Float64')
      --Motors=simROS.subscribe('/MotorsCommand', 'std_msgs/MultiArrayLayout', 'subscriber_cmd_motor_callback')
   end
end

function sysCall_actuation()
   -- Send an updated simulation time message, send the transform of the central axis
   -- and send the angle of the central axis
   print("hello")
   objectName="DDBoat_3D"
   objectHandle=sim.getObjectHandle(objectName)
   referenceName="Plane"
   referenceHandle=sim.getObjectHandle(referenceName)
   if rosInterfacePresent then
      simROS.publish(GPS,getGPS(objectName))
      simROS.publish(Compass,getMagField(objectHandle,objectName,referenceHandle,referenceName))
      simROS.publish(IMU,getImu(objectName))
      
      -- send a TF  :  robot w.r.t. floor
      simROS.sendTransform(getTransformStamped(objectHandle,objectName,referenceHandle,referenceName))
      -- To send several transforms at once, use simROS.sendTransforms instead
   end
end

function sysCall_cleanup()
   -- Following not really needed in a simulation script (i.e. automatically shut down
   -- at simulation end):
    if rosInterfacePresent then
        simROS.shutdownPublisher(GPS)
        simROS.shutdownPublisher(Camera)
        simROS.shutdownPublisher(Compass)
        simROS.shutdownPublisher(IMU)
        
        simROS.shutdownSubscriber(Motors)
        --simROS.shutdownPublisher(EncoderLeft)
        --simROS.shutdownPublisher(EncoderRight)
        --simROS.shutdownSubscriber(subscriber1)
    end
end
