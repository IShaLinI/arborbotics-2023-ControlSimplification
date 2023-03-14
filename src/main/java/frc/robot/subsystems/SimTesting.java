// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ejml.ops.FConvertArrays;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.math.FalconUtil;
import frc.robot.util.simulation.SimpleFalconSim;

public class SimTesting extends SubsystemBase {
  
  private WPI_TalonFX mFalcon500;
  private TalonFXSimCollection mFalcon500SimCollection;

  private SimpleFalconSim mFalconSim;

  double gearing = 1;

  public SimTesting() {

    mFalcon500 = new WPI_TalonFX(99);
    mFalcon500SimCollection = new TalonFXSimCollection(mFalcon500);

    mFalconSim = new SimpleFalconSim(
      mFalcon500SimCollection,
      new double[]{
        0, //kS
        FalconUtil.FALCON_KV, //kV
        0.005 //kA
      },
      gearing,
      this::getMotorSet
    );

  }

  public void set(double input){
    mFalcon500.set(input);
  }

  public double getMotorSet(){
    return mFalcon500.get();
  }

  public double getOutputRPM(){
    return FalconUtil.falconCountsToRPM(mFalcon500.getSelectedSensorVelocity(), gearing);
  }

  @Override
  public void periodic() {
    mFalconSim.update();
    Logger.getInstance().recordOutput("Sim/FalconRPM", getOutputRPM());
  }
}
