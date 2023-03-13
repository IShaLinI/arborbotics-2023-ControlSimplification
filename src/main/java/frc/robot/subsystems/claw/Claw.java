package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ClawConstants.State;

public class Claw extends SubsystemBase{

    private final WPI_TalonSRX mLeftMotor;
    private final WPI_TalonSRX mRightMotor;

    private final DoubleSolenoid mLeftPiston;
    private final DoubleSolenoid mRightPiston;

    private State mCurrentState;

    public Claw() {
        
        mLeftMotor = new WPI_TalonSRX(CAN.kLeftClaw);
        mRightMotor = new WPI_TalonSRX(CAN.kRightClaw);

        mLeftPiston = new DoubleSolenoid(CAN.kPCM, PneumaticsModuleType.REVPH, ClawConstants.kLeftPistonForward, ClawConstants.kLeftPistonReverse);
        mRightPiston = new DoubleSolenoid(CAN.kPCM, PneumaticsModuleType.REVPH, ClawConstants.kRightPistonForward, ClawConstants.kRightPistonReverse);

        mCurrentState = State.STARTING;

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

    public void setMotors(double percent) {
        mLeftMotor.set(percent);
        mRightMotor.set(percent);
    }

    public void setPistons(Value state){
        mLeftPiston.set(state);
        mRightPiston.set(state);
    }

    public void stop() {
        mLeftMotor.stopMotor();
        mRightMotor.stopMotor();
    }

    public void runClaw(){
        setMotors(mCurrentState.speed);
        setPistons(mCurrentState.value);
    }

    public Command changeState(State state){
        return new InstantCommand(() -> mCurrentState = state);
    }

    @Override
    public void periodic() {

        runClaw();

        Logger.getInstance().recordOutput("Claw/Left Voltage", mLeftMotor.getMotorOutputVoltage());
        Logger.getInstance().recordOutput("Claw/Right Voltage", mRightMotor.getMotorOutputVoltage());
        Logger.getInstance().recordOutput("Claw/State", mCurrentState.name());

    }

}

