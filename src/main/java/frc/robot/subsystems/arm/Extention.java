package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Conversions;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ExtentionConstants;
import frc.robot.Constants.RobotConstants;

public class Extention extends SubsystemBase {
    
    private final WPI_TalonFX mMotor;
    private final PIDController mPID;
    private double mTargetDistance;
    
    boolean mPIDEnabled;

    public Extention() {

        mMotor = new WPI_TalonFX(CAN.kArm);
        mPID = ExtentionConstants.kPID;
        mTargetDistance = 0;

        configureMotor();

    }

    public void configureMotor() {
        mMotor.configFactoryDefault();
        mMotor.setNeutralMode(NeutralMode.Brake);
        mMotor.setInverted(true);
        mMotor.configVoltageCompSaturation(RobotConstants.kMaximumVoltage);
        mMotor.enableVoltageCompensation(true);
        mMotor.configSupplyCurrentLimit(ExtentionConstants.kCurrentLimit);
    }

    public void set(double percent) {
        mMotor.set(percent);
    }

    public void changeSetpointMeters(double setpoint) {
        mTargetDistance = setpoint;
    }

    public void changeSetpointInches(double setpoint) {
        changeSetpointMeters(Units.inchesToMeters(setpoint));
    }

    public double getCurrentDistance() {
        return Conversions.falconCountsToMeters(mMotor.getSelectedSensorPosition(), ExtentionConstants.kGearing, ExtentionConstants.kSpoolDiameter);
    }

    public void runArm() {
        if (mPIDEnabled) {
            double PIDEffort = mPID.calculate(getCurrentDistance(), MathUtil.clamp(mTargetDistance, ExtentionConstants.kMinDistance, ExtentionConstants.kMaxDistance));
            mMotor.set(PIDEffort / 12);
        }
     }

    public void enablePID()  {mPIDEnabled = true; }
    public void disablePID() {mPIDEnabled = false;}

    public void stop() {
        mMotor.stopMotor();
    }

    public void resetEncoder() {
        mMotor.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {

        runArm();

        Logger.getInstance().recordOutput("Extention/Extention Distance", getCurrentDistance());
        Logger.getInstance().recordOutput("Extention/Extention Setpoint", mTargetDistance);
        Logger.getInstance().recordOutput("Extention/Motor Voltage", mMotor.getMotorOutputVoltage());
        Logger.getInstance().recordOutput("Extention/PID Enabled", mPIDEnabled);

    }

    public Command changeSetpoint(double distance) {
        return new InstantCommand(
            () -> mTargetDistance = distance,
            this
        );
    }

}
