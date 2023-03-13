package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ExtentionConstants;
import frc.robot.Constants.RobotConstants;

public class Extention extends SubsystemBase {
    
    private final WPI_TalonFX mMotor;
    private double mTargetDistance;
    
    boolean mPIDEnabled;

    public Extention() {
        mMotor = new WPI_TalonFX(CAN.kArm);
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

    public void stop() {
        mMotor.stopMotor();
    }

    public void resetEncoder() {
        mMotor.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {

        Logger.getInstance().recordOutput("Extention/Extention Setpoint", mTargetDistance);
        Logger.getInstance().recordOutput("Extention/Motor Voltage", mMotor.getMotorOutputVoltage());
        Logger.getInstance().recordOutput("Extention/PID Enabled", mPIDEnabled);

    }

}
