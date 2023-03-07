package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase{

    private final WPI_TalonSRX mLeftMotor;
    private final WPI_TalonSRX mRightMotor;

    private final DoubleSolenoid mLeftPiston = new DoubleSolenoid(Constants.CAN.kPCM, PneumaticsModuleType.REVPH, 12, 13);
    private final DoubleSolenoid mRightPiston = new DoubleSolenoid(Constants.CAN.kPCM, PneumaticsModuleType.REVPH, 8, 9);


    public Claw() {
        
        mLeftMotor = new WPI_TalonSRX(Constants.CAN.kLeftClaw);
        mRightMotor = new WPI_TalonSRX(Constants.CAN.kRightClaw);

        configureMotors();
    }

    public void configureMotors() {

        mLeftMotor.configFactoryDefault();
        mRightMotor.configFactoryDefault();
        
        SupplyCurrentLimitConfiguration currentLimit = new SupplyCurrentLimitConfiguration(true, 10, 10, 0);

        mLeftMotor.configSupplyCurrentLimit(currentLimit);
        mRightMotor.configSupplyCurrentLimit(currentLimit);
        
    }

    public void intake() {
        mLeftMotor.set(-0.4);
        mRightMotor.set(0.4);
    }

    public void outtake() {
        mLeftMotor.set(0.4);
        mRightMotor.set(-0.4);
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
    
}

