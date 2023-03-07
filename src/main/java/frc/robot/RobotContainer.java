package frc.robot;

import frc.robot.subsystems.arm.Extention;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.Drivetrain;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.custom.controls.Deadbander;

public class RobotContainer {

  private CommandXboxController mDriver = new CommandXboxController(0);
  private CommandXboxController mOperator = new CommandXboxController(1);

  private SlewRateLimiter mXTranslationLimiter = DriveConstants.kXTranslationLimiter;
  private SlewRateLimiter mYTranslationLimiter = DriveConstants.kYTranslationLimiter;
  private SlewRateLimiter mRotationLimiter = DriveConstants.kRotationLimiter;

  private final Drivetrain mDrivetrain = new Drivetrain();
  private final Extention mExtention = new Extention();
  private final Claw mClaw = new Claw();
  private final Pivot mPivot = new Pivot();

  private final AutoCommands mAutos = new AutoCommands(mClaw, mDrivetrain, mExtention, mPivot);

  private SendableChooser<SequentialCommandGroup[]> mAutoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
  }

 
  private void configureBindings() {

    mDrivetrain.setDefaultCommand(
      new RunCommand(
        () -> mDrivetrain.drive(
          -mXTranslationLimiter.calculate(Deadbander.applyLinearScaledDeadband(mDriver.getLeftY(), 0.1) * DriveConstants.kMaxTranslationSpeed), 
          -mYTranslationLimiter.calculate(Deadbander.applyLinearScaledDeadband(mDriver.getLeftX(), 0.1) * DriveConstants.kMaxTranslationSpeed), 
          -mRotationLimiter.calculate(Deadbander.applyLinearScaledDeadband(mDriver.getRightX(), 0.1) * DriveConstants.kMaxRotationSpeed), 
          true
        )
        , mDrivetrain
      )
    );

    mPivot.setDefaultCommand(
      new RunCommand(
        () -> mPivot.trimTargetAngle(Deadbander.applyLinearScaledDeadband(-mOperator.getLeftY(), 0.2) * 0.3), 
        mPivot
      )
    );

    mExtention.setDefaultCommand(
      new RunCommand(
        () -> mExtention.set(Deadbander.applyLinearScaledDeadband(-mOperator.getRightY(), 0.1)), 
        mExtention
      )
    );

    mOperator.a().whileTrue(
      mPivot.changeSetpoint(PivotConstants.SETPOINTS.INTAKE).alongWith(
      mClaw.changeState(ClawConstants.State.GRAB)
    ));

    mOperator.b().whileTrue(
      mClaw.changeState(ClawConstants.State.RELEASE)
    );

    mOperator.povUp().whileTrue(
      mPivot.changeSetpoint(PivotConstants.SETPOINTS.SCORE)
    );

    //Consider automating carry state after intaking
    mOperator.povRight().whileTrue(
      mPivot.changeSetpoint(PivotConstants.SETPOINTS.CARRY)
    );

  }

  public Command getAutonomousCommand() {
    return mAutos.getAuto();
  }

}