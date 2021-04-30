function init()
    simROS.subscribe('car_vel2','std_msgs/Float32MultiArray','setMotorVelocity')
    simROS.subscribe('car_torque2','std_msgs/Float32MultiArray','setMotorTorque')
end


function setMotorVelocity(msg)
    -- print('hello from setMotorVelocity')

    motorHandles={-1,-1,-1,-1}

    motorHandles[1]=sim.getObjectHandle('joint_front_left_wheel')
    motorHandles[2]=sim.getObjectHandle('joint_front_right_wheel')
    motorHandles[3]=sim.getObjectHandle('joint_back_right_wheel')
    motorHandles[4]=sim.getObjectHandle('joint_back_left_wheel')

    des_vel = msg.data

    sim.setJointTargetVelocity(motorHandles[1],des_vel[1] - des_vel[2])
    sim.setJointTargetVelocity(motorHandles[2],-des_vel[1] - des_vel[2])
    sim.setJointTargetVelocity(motorHandles[3],-des_vel[1] - des_vel[2])
    sim.setJointTargetVelocity(motorHandles[4],des_vel[1] - des_vel[2])

end

maxVel = 999999
function setMotorTorque(msg)
    motorHandles={-1, -1, -1, -1}

    motorHandles[1]=sim.getObjectHandle('joint_front_left_wheel')
    motorHandles[2]=sim.getObjectHandle('joint_front_right_wheel')
    motorHandles[3]=sim.getObjectHandle('joint_back_right_wheel')
    motorHandles[4]=sim.getObjectHandle('joint_back_left_wheel')

    des_torque = msg.data
    left_vel = maxVel
    right_vel = maxVel
    if des_torque[1] < 0 then
        left_vel = -maxVel
    end
    if des_torque[2] < 0 then
        right_vel = -maxVel
    end

    sim.setJointMaxForce(motorHandles[1], des_torque[1])
    sim.setJointMaxForce(motorHandles[2], des_torque[2])
    sim.setJointMaxForce(motorHandles[3], des_torque[2])
    sim.setJointMaxForce(motorHandles[4], des_torque[1])

    sim.setJointTargetVelocity(motorHandles[1], left_vel)
    sim.setJointTargetVelocity(motorHandles[2], -right_vel)
    sim.setJointTargetVelocity(motorHandles[3], -right_vel)
    sim.setJointTargetVelocity(motorHandles[4], left_vel)
end

function sysCall_threadmain()
    init()

    motorHandles={-1,-1,-1,-1}

    motorHandles[1]=sim.getObjectHandle('joint_front_left_wheel')
    motorHandles[2]=sim.getObjectHandle('joint_front_right_wheel')
    motorHandles[3]=sim.getObjectHandle('joint_back_right_wheel')
    motorHandles[4]=sim.getObjectHandle('joint_back_left_wheel')

    -- Prior used for testing...
    -- sim.setJointMaxForce(motorHandles[1], .005)
    -- sim.setJointMaxForce(motorHandles[2], .005)
    -- sim.setJointMaxForce(motorHandles[3], .005)
    -- sim.setJointMaxForce(motorHandles[4], .005)

    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        -- sim.setJointTargetVelocity(motorHandles[1], maxVel)
        -- sim.setJointTargetVelocity(motorHandles[2], -maxVel)
        -- sim.setJointTargetVelocity(motorHandles[3], -maxVel)
        -- sim.setJointTargetVelocity(motorHandles[4], maxVel)
    end

    --  for i=1,4,1 do
    --     print("YEET")
    --     sim.setJointTargetVelocity(motorHandles[1],80*math.pi/180)
    --     sim.setJointTargetVelocity(motorHandles[2],-80*math.pi/180)
    --     sim.setJointTargetVelocity(motorHandles[3],-80*math.pi/180)
    --     sim.setJointTargetVelocity(motorHandles[4],80*math.pi/180)
    --  end
        
    -- Stop motors on shutdown
    sim.setJointTargetVelocity(motorHandles[1],0)
    sim.setJointTargetVelocity(motorHandles[2],0)
    sim.setJointTargetVelocity(motorHandles[3],0)
    sim.setJointTargetVelocity(motorHandles[4],0)
end