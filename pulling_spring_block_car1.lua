function init()
    --print('hello')
    moduleName=0
    moduleVersion=0
    index=0
    pluginNotFound=true
    while moduleName do
        moduleName,moduleVersion=sim.getModuleName(index)
        if (moduleName=='ROSInterface') then
            pluginNotFound=false
        end
        index=index+1
    end
    

    if (not pluginNotFound) then
        TensionPub = simROS.advertise('tensions','std_msgs/Float32MultiArray')
        PositionPub = simROS.advertise('pos', 'std_msgs/Float32MultiArray')
        -- PositionPub1 = simROS.advertise('car_pos1', 'std_msgs/Float32MultiArray')
        -- PositionPub2 = simROS.advertise('block_pos', 'std_msgs/Float32MultiArray')
        -- PositionPub3 = simROS.advertise('car_pos2', 'std_msgs/Float32MultiArray')

        BlockAnglePub = simROS.advertise('block_angle', 'std_msgs/Float32MultiArray')
        simROS.subscribe('car_vel1','std_msgs/Float32MultiArray','setMotorVelocity')
        simROS.subscribe('car_torque1','std_msgs/Float32MultiArray','setMotorTorque')
        --CarSub2=simROS.subscribe('car_vel2','std_msgs/Float32MultiArray','setMotorVelocity2')
        --TorquePub = simROS.advertise('torques','std_msgs/Float32MultiArray')
        --CopterSub=simROS.subscribe('quadPos', 'std_msgs/Float32MultiArray', 'setQuadSpeeds')
        --EulerPub = simROS.advertise('euler', 'std_msgs/Float32MultiArray')
    else
        print("<font color='#F00'>ROS interface was not found. Cannot run.</font>@html")
    end

end

function getTensions(sensor1, sensor2, sensor3, sensor4)

    r1, tension1, torque1 = sim.readForceSensor(sensor1)
    r2, tension2, torque2 = sim.readForceSensor(sensor2)
    r3, tension3, torque3 = sim.readForceSensor(sensor3)
    r4, tension4, torque4 = sim.readForceSensor(sensor4)
 
    if (r1~=0 and r2~=0 and r3~=0 and r4~=0) then
        simTime = sim.getSimulationTime()
        tensionMatrix = {tension1[3], 
                         tension2[3], 
                         tension3[3], 
                         tension4[3],
                         simTime}
        tensions = {layout={}, data=tensionMatrix}
        return tensions
    else
        return nil
    end


end

function getPositions(car1, block, car2)

    p1 = sim.getObjectPosition(car1,-1)
    p2 = sim.getObjectPosition(block,-1)
    p3 = sim.getObjectPosition(car2,-1)

    if (p1~=-1 and p2~=-1 and p3~=-1) then
        simTime = sim.getSimulationTime()
        -- p1[#p1+1] = simTime
        -- p2[#p2+1] = simTime
        -- p3[#p3+1] = simTime
        -- table.concat(p1 ,p2)
        -- pos1 = {layout={}, data=p1}
        -- pos2 = {layout={}, data=p2}
        -- pos3 = {layout={}, data=p3}
        data = {p1[1], p1[2], p1[3],
               p2[1], p2[2], p2[3],
               p3[1], p3[2], p3[3],
               simTime}
        -- p = {pos1, pos2, pos3}
        pos = {layout={}, data=data}
        return pos
    else
        return nil
    end

end

function getAngles(block)

    p = sim.getObjectOrientation(block,-1)

    if (p~=-1) then
        simTime = sim.getSimulationTime()
        p[#p+1] = simTime
        angle = {layout={}, data=p}
        --print(angle)
        return angle
    else
        return nil
    end

end

-- car_vel subscriber callback
function setMotorVelocity(msg)

    motorHandles={-1,-1,-1,-1}

    motorHandles[1]=sim.getObjectHandle('joint_front_left_wheel')
    motorHandles[2]=sim.getObjectHandle('joint_front_right_wheel')
    motorHandles[3]=sim.getObjectHandle('joint_back_right_wheel')
    motorHandles[4]=sim.getObjectHandle('joint_back_left_wheel')

    des_vel = msg.data
    print(des_vel)

    sim.setJointTargetVelocity(motorHandles[1],-des_vel[1] - des_vel[2])
    sim.setJointTargetVelocity(motorHandles[2],des_vel[1] - des_vel[2])
    sim.setJointTargetVelocity(motorHandles[3],des_vel[1] - des_vel[2])
    sim.setJointTargetVelocity(motorHandles[4],-des_vel[1] - des_vel[2])

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
    
    sim.setThreadAutomaticSwitch(true) -- disable automatic thread switches
    sensor1 = sim.getObjectHandle('Force_sensor1')
    sensor2 = sim.getObjectHandle('Force_sensor')
    sensor3 = sim.getObjectHandle('Force_sensor0')
    sensor4 = sim.getObjectHandle('Force_sensor2')

    car1 = sim.getObjectHandle('Robotnik_Summit_XL')
    block = sim.getObjectHandle('Cuboid')
    car2 = sim.getObjectHandle('Robotnik_Summit_XL#0')    

    --motorHandles={-1,-1,-1,-1}

    --motorHandles[1]=sim.getObjectHandle('joint_front_left_wheel')
    --motorHandles[2]=sim.getObjectHandle('joint_front_right_wheel')
    --motorHandles[3]=sim.getObjectHandle('joint_back_right_wheel')
    --motorHandles[4]=sim.getObjectHandle('joint_back_left_wheel')
    

    --for i=1,4,1 do
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        --sim.setJointTargetVelocity(motorHandles[1],80*math.pi/180)
        --sim.setJointTargetVelocity(motorHandles[2],-80*math.pi/180)
        --sim.setJointTargetVelocity(motorHandles[3],-80*math.pi/180)
        --sim.setJointTargetVelocity(motorHandles[4],80*math.pi/180)

        t = getTensions(sensor1, sensor2, sensor3, sensor4)
        
        if (t~=nil) then
            --print(t)
            --print(type(TensionPub))
            simROS.publish(TensionPub, t)
        end

        p = getPositions(car1, block, car2)
        if (p~=nil) then
            simROS.publish(PositionPub, p)
            -- simROS.publish(PositionPub1, p[1])
            -- simROS.publish(PositionPub2, p[2])
            -- simROS.publish(PositionPub3, p[3])
        end
        
        a = getAngles(block)
        if (a~=nil) then
            --print(type(a))
            simROS.publish(BlockAnglePub, a)
        end
        
        --sim.switchThread()
     end
        
    -- sim.setJointTargetVelocity(motorHandles[1],1)
    -- sim.setJointTargetVelocity(motorHandles[2],-1)
    -- sim.setJointTargetVelocity(motorHandles[3],-1)
    -- sim.setJointTargetVelocity(motorHandles[4],1)
end