package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ExtentionConstants;
import frc.robot.Constants.FalconConstants;

public class Extention extends SubsystemBase {
    
    private final WPI_TalonFX mArm;
    private final PIDController mPID;
    private double mTargetExtension;
    
    boolean mPIDEnabled;

    public Extention() {

        mArm = new WPI_TalonFX(Constants.CAN.kArm);
        mArm.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 0));
        mArm.setInverted(true);
        mPID = ExtentionConstants.kPID;

        configureMotor();

    }

    public void configureMotor() {
        mArm.configFactoryDefault();
        mArm.setNeutralMode(NeutralMode.Brake);
        mArm.configVoltageCompSaturation(10);
        mArm.enableVoltageCompensation(true);
        mArm.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 10, 0));
    }

    public void set(double percent) {
        mArm.set(percent);
    }

    public void changeSetpointMeters(double setpoint) {
        mTargetExtension = setpoint;
    }

    public void changeSetpointInches(double setpoint) {
        changeSetpointMeters(Units.inchesToMeters(setpoint));
    }

    public double getCurrentDistance() {
        return FalconConstants.falconCountsToMeters(mArm.getSelectedSensorPosition(), ExtentionConstants.kGearing, ExtentionConstants.kSpoolDiameter);
    }

    public void runArm() {

        if (mPIDEnabled) {

            double PIDEffort = mPID.calculate(getCurrentDistance(), MathUtil.clamp(mTargetExtension, 0, Units.inchesToMeters(35)));
            mArm.set(PIDEffort / 12);

        }

     }

     public void enablePID()  {mPIDEnabled = true; }
     public void disablePID() {mPIDEnabled = false;}

    public void extend() {
        mArm.set(0.6);
    }

    public void retract() {
        mArm.set(-0.6);
    }

    public void stop() {
        mArm.stopMotor();;
    }

    public void resetEncoder() {
        mArm.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {

        runArm();

        Logger.getInstance().recordOutput("Raw Encoder", mArm.getSelectedSensorPosition());
        Logger.getInstance().recordOutput("Arm Meters", getCurrentDistance());
        Logger.getInstance().recordOutput("Extension Setpoint", mTargetExtension);
        Logger.getInstance().recordOutput("extention volts", mArm.getMotorOutputVoltage());
    }

}
