function init()
    CarSub2=simROS.subscribe('car_vel2','std_msgs/Float32MultiArray','setMotorVelocity')
end


function setMotorVelocity(msg)
    print('hello from setMotorVelocity')

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

function sysCall_threadmain()
    init()

    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
    end
    --motorHandles={-1,-1,-1,-1}

    --motorHandles[1]=sim.getObjectHandle('joint_front_left_wheel')
    --motorHandles[2]=sim.getObjectHandle('joint_front_right_wheel')
    --motorHandles[3]=sim.getObjectHandle('joint_back_right_wheel')
    --motorHandles[4]=sim.getObjectHandle('joint_back_left_wheel')

     --for i=1,4,1 do
        --sim.setJointTargetVelocity(motorHandles[1],80*math.pi/180)
        --sim.setJointTargetVelocity(motorHandles[2],-80*math.pi/180)
        --sim.setJointTargetVelocity(motorHandles[3],-80*math.pi/180)
        --sim.setJointTargetVelocity(motorHandles[4],80*math.pi/180)
     --end
        
    -- sim.setJointTargetVelocity(motorHandles[1],1)
    -- sim.setJointTargetVelocity(motorHandles[2],-1)
    -- sim.setJointTargetVelocity(motorHandles[3],-1)
    -- sim.setJointTargetVelocity(motorHandles[4],1)
end