package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.PivotConstants.SETPOINTS;
import frc.robot.util.math.FalconUtil;

public class Pivot extends SubsystemBase {
    
    private final WPI_TalonFX mPivot;

    private final DoubleSolenoid mRatchet;
    private final DutyCycleEncoder mEncoder;
    private final PIDController mPID;

    public double mTargetAngle;
    public double mFalconOffset;
    public double mTrimAngle = 0;

    public boolean mFirstEngage = true;

    public Pivot() {

        mPivot = new WPI_TalonFX(CAN.kPivot);

        mRatchet = new DoubleSolenoid(
            CAN.kPCM, 
            PneumaticsModuleType.REVPH, 
            PivotConstants.kRatchetForward, 
            PivotConstants.kRatchetReverse
        );

        mEncoder = new DutyCycleEncoder(PivotConstants.kThroughboreDIO);
        mPID = PivotConstants.kPID;
        Timer.delay(1);
        mEncoder.setPositionOffset(PivotConstants.kThroughboreOffset);
        mPID.setTolerance(PivotConstants.kPositionTollerance, PivotConstants.kVelocotiyTollerance);

        zeroEncoder();
        configureMotor();

    }

    public void configureMotor() {
        mPivot.configFactoryDefault();
        mPivot.setNeutralMode(NeutralMode.Brake);
        mPivot.setInverted(InvertType.InvertMotorOutput);
        mPivot.setSelectedSensorPosition(0);
        mPivot.configVoltageCompSaturation(RobotConstants.kMaximumVoltage);
        mPivot.enableVoltageCompensation(true);
        mPivot.configSupplyCurrentLimit(PivotConstants.kCurrentLimit);
    }

    public void runPivot() {

        if(atTarget()) {
            engageRatchet();
            mPivot.set(0);
            mFirstEngage = true;
        }else{
            disengageRatchet();

            Timer.delay(mFirstEngage ? 0.05 : 0);

            double mPIDEffort = mPID.calculate(
                getAngle(), 
                MathUtil.clamp(
                    mTargetAngle + mTrimAngle,
                    PivotConstants.kMinAngle, 
                    PivotConstants.kMaxAngle
                )
            );

            mPivot.set(MathUtil.clamp(mPIDEffort / 12, -1, 1));

            mFirstEngage = false;

        }

    }

    public void zeroEncoder() {
        mFalconOffset = FalconUtil.degreesToFalconCounts(getThroughBoreAngle(), PivotConstants.kGearing);
    }

    public boolean atTarget(){
        return Math.abs(getAngle() - mTargetAngle + mTrimAngle) < PivotConstants.kPositionTollerance;
    }

    /**
     * @param angle in degrees
     */
    public void trimTargetAngle(double angle){
        mTrimAngle = angle;
    }

    public void engageRatchet() {
        mRatchet.set(Value.kForward);
    }

    public void disengageRatchet() {
        mRatchet.set(Value.kReverse);
    }
  
    public void stop() {
        mPivot.stopMotor();
    }

    public double getAngle() {
        return FalconUtil.degreesToFalconCounts(mPivot.getSelectedSensorPosition() + mFalconOffset, PivotConstants.kGearing);
    }

    public double getThroughBoreAngle() {
        return ((mEncoder.getAbsolutePosition()) - mEncoder.getPositionOffset()) * 360;
    }

    @Override
    public void periodic() {

        runPivot();

        Logger.getInstance().recordOutput("Pivot/TB Raw", mEncoder.getAbsolutePosition());
        Logger.getInstance().recordOutput("Pivot/TB Offset", mEncoder.getAbsolutePosition() - mEncoder.getPositionOffset());
        Logger.getInstance().recordOutput("Pivot/TB Angle", getThroughBoreAngle());
        Logger.getInstance().recordOutput("Pivot/Setpoint Angle", mTargetAngle + mTrimAngle);
        Logger.getInstance().recordOutput("Pivot/Current Angle", getAngle());
        Logger.getInstance().recordOutput("Pivot/At Target", atTarget());
        Logger.getInstance().recordOutput("Pivot/Motor Voltage", mPivot.getMotorOutputVoltage());
        Logger.getInstance().recordOutput("Pivot/Motor Set", mPivot.get());
        Logger.getInstance().recordOutput("Pivot/Ratchet State", (mRatchet.get() == Value.kForward) ? "Engaged" : "Disengaged");
        Logger.getInstance().recordOutput("Pivot/Trim Value", mTrimAngle);

    }

    public Command changeSetpoint(SETPOINTS setpoint) {
        return new InstantCommand(
            () -> mTargetAngle = setpoint.angle,
            this
        ).andThen(
            new InstantCommand(
                () -> Logger.getInstance().recordOutput("Pivot/Setpoint", setpoint.name()),
                this
            )
        );
    }
    
}