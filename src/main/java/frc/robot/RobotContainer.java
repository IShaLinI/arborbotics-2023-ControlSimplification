package frc.robot;

import frc.robot.subsystems.arm.Extention;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.Drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
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

  private SendableChooser<SequentialCommandGroup[]> mAutoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
  }

 
  private void configureBindings() {

    mDrivetrain.setDefaultCommand(
      mDrivetrain.DriverControl(
        () -> -mXTranslationLimiter.calculate(Deadbander.applyLinearScaledDeadband(mDriver.getLeftY(), 0.1) * DriveConstants.kMaxTranslationSpeed), 
        () -> -mYTranslationLimiter.calculate(Deadbander.applyLinearScaledDeadband(mDriver.getLeftX(), 0.1) * DriveConstants.kMaxTranslationSpeed), 
        () -> -mRotationLimiter.calculate(Deadbander.applyLinearScaledDeadband(mDriver.getRightX(), 0.1) * DriveConstants.kMaxRotationSpeed), 
        true
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

    configureAutoChooser();

  }

      
  public void configureAutoChooser() {
    mAutoChooser.setDefaultOption("Nothing", new SequentialCommandGroup[]{null, null});
    SmartDashboard.putData("Auto Chooser", mAutoChooser);
  }

  public SequentialCommandGroup getAutonomousCommand() {
    int alliance = (DriverStation.getAlliance() == Alliance.Blue) ? 0 : 1;
    Logger.getInstance().recordOutput("Robot/SelectedAlliance", alliance);
    return mAutoChooser.getSelected()[alliance];
  }

}