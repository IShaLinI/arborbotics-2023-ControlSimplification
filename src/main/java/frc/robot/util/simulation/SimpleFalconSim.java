// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.simulation;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Class to simulate a very simple Falcon500 motor based system.
 * 
 * Author: Andrew Card
 * 
*/
public class SimpleFalconSim {

    private TalonFXSimCollection mFalconSim;
    private FlywheelSim mOutputShaftSim;

    private static double mGearRatio;

    private Supplier<Double> mMotorSet;

    private static double dtSeconds = 0.02;


    /**
     * @param _falconSim Motor Sim Collection
     * @param _sysid //[kv, kA]
     * @param _gearRatio //Output ratio
     * @param _motorSet //Motor set -1 to 1
     */
    public SimpleFalconSim(
        TalonFXSimCollection _falconSim,
        double[] _sysid,
        double _gearRatio,
        Supplier<Double> _motorSet
    ){
        mFalconSim = _falconSim;
        mGearRatio = _gearRatio;

        LinearSystem<N1,N1,N1> _plant = LinearSystemId.identifyVelocitySystem(_sysid[1] / 12, _sysid[2] / 12);

        mOutputShaftSim = new FlywheelSim(_plant, DCMotor.getFalcon500(1), _gearRatio);

        mMotorSet = _motorSet;
    
    }

    public void update(){
        mOutputShaftSim.setInput(mMotorSet.get() * RobotController.getBatteryVoltage());
        mOutputShaftSim.update(dtSeconds);

        double shaftVelocity = (mOutputShaftSim.getAngularVelocityRPM() * mGearRatio * 2048) / 600;
        double shaftPositionDelta = shaftVelocity * 10 * dtSeconds;

        mFalconSim.setIntegratedSensorVelocity((int)shaftPositionDelta);
        mFalconSim.addIntegratedSensorPosition((int)shaftPositionDelta);
    }

}
