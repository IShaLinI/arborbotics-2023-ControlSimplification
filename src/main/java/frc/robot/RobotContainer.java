package frc.robot;

import frc.robot.commands.DriverControl;
import frc.robot.subsystems.arm.Extention;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.Drivetrain;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.custom.controls.deadbander;

public class RobotContainer {

  private CommandXboxController mDriver = new CommandXboxController(0);
  private CommandXboxController mOperator = new CommandXboxController(1);

  private static final Field2d mField = new Field2d();

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
      new DriverControl(mDrivetrain, 
        ()-> deadbander.applyLinearScaledDeadband(-mDriver.getRightY(), 0.1) * 2.5, 
        ()-> deadbander.applyLinearScaledDeadband(-mDriver.getRightX(), 0.1) * 2.5, 
        ()-> deadbander.applyLinearScaledDeadband(-mDriver.getLeftX(), 0.1) * 2.5, 
        true
      )
    );

    mPivot.setDefaultCommand(
      new RunCommand(
        () -> mPivot.trimTargetAngle(deadbander.applyLinearScaledDeadband(-mOperator.getLeftY(), 0.2) * 0.3), 
        mPivot
      )
    );

    mExtention.setDefaultCommand(
      new RunCommand(
        () -> mExtention.set(deadbander.applyLinearScaledDeadband(-mOperator.getRightY(), 0.1)), 
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
        
        int alliance = 0;
        if(DriverStation.getAlliance() == Alliance.Blue){
          alliance = 0;
        }else{
          alliance = 1;
        }

         return mAutoChooser.getSelected()[alliance];
      }

      public static Field2d getField() {
        return mField;
      }

    }