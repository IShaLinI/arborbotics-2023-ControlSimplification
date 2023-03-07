package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.RobotConstants;

public class Claw extends SubsystemBase{

    private final WPI_TalonSRX mLeftMotor;
    private final WPI_TalonSRX mRightMotor;

    private final DoubleSolenoid mLeftPiston;
    private final DoubleSolenoid mRightPiston;

    public Claw() {
        
        mLeftMotor = new WPI_TalonSRX(CAN.kLeftClaw);
        mRightMotor = new WPI_TalonSRX(CAN.kRightClaw);

        mLeftPiston = new DoubleSolenoid(CAN.kPCM, PneumaticsModuleType.REVPH, ClawConstants.kLeftPistonForward, ClawConstants.kLeftPistonReverse);
        mRightPiston = new DoubleSolenoid(CAN.kPCM, PneumaticsModuleType.REVPH, ClawConstants.kRightPistonForward, ClawConstants.kRightPistonReverse);

        configureMotors();
    }

    public void configureMotors() {

        mLeftMotor.configFactoryDefault();
        mRightMotor.configFactoryDefault();

        mLeftMotor.configSupplyCurrentLimit(ClawConstants.kCurrentLimit);
        mRightMotor.configSupplyCurrentLimit(ClawConstants.kCurrentLimit);

        mLeftMotor.configVoltageCompSaturation(RobotConstants.kMaximumVoltage);
        mRightMotor.configVoltageCompSaturation(RobotConstants.kMaximumVoltage);

        mLeftMotor.enableVoltageCompensation(true);
        mRightMotor.enableVoltageCompensation(true);

        mLeftMotor.setInverted(false);
        mRightMotor.setInverted(true);
        
    }

    public void intake() {
        mLeftMotor.set(ClawConstants.kSpeed);
        mRightMotor.set(ClawConstants.kSpeed);
    }

    public void outtake() {
        mLeftMotor.set(ClawConstants.kSpeed);
        mRightMotor.set(ClawConstants.kSpeed);
    }

    public void open(){
        mLeftPiston.set(Value.kForward);
        mRightPiston.set(Value.kForward);
    }

    public void close(){
        mRightPiston.set(Value.kReverse);
        mRightPiston.set(Value.kReverse);
    }

    public void stop() {
        mLeftMotor.stopMotor();
        mRightMotor.stopMotor();
    }

    @Override
    public void periodic() {
        Logger.getInstance().recordOutput("Claw/Left Voltage", mLeftMotor.getMotorOutputVoltage());
        Logger.getInstance().recordOutput("Claw/Right Voltage", mRightMotor.getMotorOutputVoltage());
        Logger.getInstance().recordOutput("Claw/State", (mLeftPiston.get() == Value.kForward) ? "Open" : "Closed");
    }
    
}

