package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.MecanumAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.arm.Extention;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.Drivetrain;


public class AutoCommands {
    private final Claw claw;
    private final Drivetrain drivetrain; 
    private final Extention extention;
    private final Pivot pivot;
    private final MecanumAutoBuilder autoBuilder;
    private final SendableChooser<Command> autoChooser;
    HashMap<String, Command> eventMap = new HashMap<String, Command>();

    public AutoCommands(Claw claw, Drivetrain drivetrain, Extention extention, Pivot pivot){
        this.claw = claw;
        this.drivetrain = drivetrain;
        this.extention = extention;
        this.pivot = pivot;

        eventMap.put("grab", new SequentialCommandGroup(
            claw.changeState(ClawConstants.State.GRAB),
            pivot.changeSetpoint(PivotConstants.SETPOINTS.INTAKE)
        ));

        eventMap.put("readyArm", new SequentialCommandGroup(
            pivot.changeSetpoint(PivotConstants.SETPOINTS.SCORE)
        ));

        eventMap.put("score", new SequentialCommandGroup(
            claw.changeState(ClawConstants.State.RELEASE)
        ));

        eventMap.put("carry", new SequentialCommandGroup(
            claw.changeState(ClawConstants.State.IDLE),
            pivot.changeSetpoint(PivotConstants.SETPOINTS.CARRY)
        ));

        autoBuilder = new MecanumAutoBuilder(
            drivetrain::getPose, 
            drivetrain::resetPoseAndGyro, 
            drivetrain.getKinematics(), 
            DriveConstants.kTrajTranslationPID, 
            DriveConstants.kTrajRotationPID,
            DriveConstants.kTrajectoryMaxSpeed,
            drivetrain::setSpeeds,
            eventMap,
            true,
            drivetrain
        );

        autoChooser = new SendableChooser<>();

        autoChooser.addOption("Nothing", new PrintCommand("No auto selected"));

        autoChooser.addOption("Demo Auto", makeAuto("PathTest"));

        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    public Command getAuto(){
        return autoChooser.getSelected();
    }

    private Command makeAuto(String path) {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup(path, DriveConstants.kTrajectoryMaxSpeed, DriveConstants.kTrajectoryMaxAccel));
    }

}
