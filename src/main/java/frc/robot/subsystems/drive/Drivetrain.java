package frc.robot.subsystems.drive;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Conversions;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.Vision;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonFX mFrontRight;
  private final WPI_TalonFX mFrontLeft;
  private final WPI_TalonFX mBackRight;
  private final WPI_TalonFX mBackLeft;

  private final WPI_Pigeon2 mPigeon;

  private final PIDController mFrontLeftPIDController;
  private final PIDController mFrontRightPIDController;
  private final PIDController mBackLeftPIDController;
  private final PIDController mBackRightPIDController;

  private final Translation2d mFrontLeftLocation;
  private final Translation2d mFrontRightLocation;
  private final Translation2d mBackLeftLocation;
  private final Translation2d mBackRightLocation;

  private final SimpleMotorFeedforward mFeedForward;

  private final MecanumDriveKinematics mKinematics;  
  private final MecanumDrivePoseEstimator mPoseEstimator;

  private final MecanumSimulation mSimulation;
  private final Vision mVision;
  
  private Field2d mField;

  public Drivetrain() {

    mPigeon = new WPI_Pigeon2(CAN.kPigeon);

    mPigeon.reset();

    mFrontLeft = new WPI_TalonFX(CAN.kFrontLeft);
    mFrontRight = new WPI_TalonFX(CAN.kFrontRight);
    mBackRight = new WPI_TalonFX(CAN.kBackRight);
    mBackLeft = new WPI_TalonFX(CAN.kBackLeft);
    
    mFrontLeftPIDController = DriveConstants.kWheelPID;
    mFrontRightPIDController = DriveConstants.kWheelPID;
    mBackLeftPIDController = DriveConstants.kWheelPID;
    mBackRightPIDController = DriveConstants.kWheelPID;

    mFrontLeftLocation = DriveConstants.kWheelPositions[0];
    mFrontRightLocation = DriveConstants.kWheelPositions[1];
    mBackLeftLocation = DriveConstants.kWheelPositions[2];
    mBackRightLocation = DriveConstants.kWheelPositions[3];

    mKinematics = new MecanumDriveKinematics(
      mFrontLeftLocation, 
      mFrontRightLocation, 
      mBackLeftLocation, 
      mBackRightLocation
    );
    
    mFeedForward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);
    
    mPoseEstimator = new MecanumDrivePoseEstimator(
      mKinematics, 
      new Rotation2d(Units.degreesToRadians(-90)),
      getCurrentDistances(), 
      new Pose2d()
    );

    mSimulation = new MecanumSimulation(
      new TalonFXSimCollection[]{
        mFrontLeft.getSimCollection(),
        mFrontRight.getSimCollection(),
        mBackLeft.getSimCollection(),
        mBackRight.getSimCollection()
      },
      mPigeon.getSimCollection(),
      DriveConstants.kDrivetrainCharacterization,
      mKinematics,
      DCMotor.getFalcon500(1), 
      KitbotGearing.k10p71.value, 
      this::getCurrentState,
      this::getMotorSets
    );

    mVision = new Vision();

    mField = new Field2d();

    SmartDashboard.putData(mField);

  }

  public void configureMotor(){

    mFrontLeft.configFactoryDefault();
    mFrontRight.configFactoryDefault();
    mBackLeft.configFactoryDefault();
    mBackRight.configFactoryDefault();

    mFrontLeft.setNeutralMode(NeutralMode.Brake);
    mFrontRight.setNeutralMode(NeutralMode.Brake);
    mBackLeft.setNeutralMode(NeutralMode.Brake);
    mBackRight.setNeutralMode(NeutralMode.Brake);

    mFrontLeft.configSupplyCurrentLimit(DriveConstants.kCurrentLimit);
    mFrontRight.configSupplyCurrentLimit(DriveConstants.kCurrentLimit);
    mBackLeft.configSupplyCurrentLimit(DriveConstants.kCurrentLimit);
    mBackRight.configSupplyCurrentLimit(DriveConstants.kCurrentLimit);

    if(RobotBase.isReal()){
      mFrontRight.setInverted(true);
      mBackRight.setInverted(true);
    }

    mFrontLeft.configVoltageCompSaturation(RobotConstants.kMaximumVoltage);
    mFrontRight.configVoltageCompSaturation(RobotConstants.kMaximumVoltage);
    mBackLeft.configVoltageCompSaturation(RobotConstants.kMaximumVoltage);
    mBackRight.configVoltageCompSaturation(RobotConstants.kMaximumVoltage);

    mFrontLeft.enableVoltageCompensation(true);
    mFrontRight.enableVoltageCompensation(true);
    mBackLeft.enableVoltageCompensation(true);
    mBackRight.enableVoltageCompensation(true);

  }

  @Override
  public void periodic() {

    updateOdometry();
    mField.setRobotPose(getPose());

    Logger.getInstance().recordOutput("Drivetrain/Front Left Position", getCurrentDistances().frontLeftMeters);
    Logger.getInstance().recordOutput("Drivetrain/Front Right Position", getCurrentDistances().frontRightMeters);
    Logger.getInstance().recordOutput("Drivetrain/Back Left Position", getCurrentDistances().rearLeftMeters);
    Logger.getInstance().recordOutput("Drivetrain/Back Right Position", getCurrentDistances().rearRightMeters);

    Logger.getInstance().recordOutput("Drivetrain/Front Left Velocity", getCurrentState().frontLeftMetersPerSecond);
    Logger.getInstance().recordOutput("Drivetrain/Front Right Velocity", getCurrentState().frontRightMetersPerSecond);
    Logger.getInstance().recordOutput("Drivetrain/Back Left Velocity", getCurrentState().rearLeftMetersPerSecond);
    Logger.getInstance().recordOutput("Drivetrain/Back Right Velocity", getCurrentState().rearRightMetersPerSecond);

    Logger.getInstance().recordOutput("Drivetrain/Front Left Voltage", mFrontLeft.getMotorOutputVoltage());
    Logger.getInstance().recordOutput("Drivetrain/Front Right Voltage", mFrontRight.getMotorOutputVoltage());
    Logger.getInstance().recordOutput("Drivetrain/Back Left Voltage", mBackLeft.getMotorOutputVoltage());
    Logger.getInstance().recordOutput("Drivetrain/Back Right Voltage", mBackRight.getMotorOutputVoltage());

    Logger.getInstance().recordOutput("Drivetrain/Gyro Angle", mPigeon.getAngle());

  }

  public void resetGyro() {

    mPigeon.reset();

  }

  // current state of dt velocity
  public MecanumDriveWheelSpeeds getCurrentState() {
    return new MecanumDriveWheelSpeeds(
      Conversions.falconCountsToMeters(mFrontLeft.getSelectedSensorVelocity() * 10, DriveConstants.kGearing, DriveConstants.kWheelDiameter), 
      Conversions.falconCountsToMeters(mFrontRight.getSelectedSensorVelocity() * 10, DriveConstants.kGearing, DriveConstants.kWheelDiameter),
      Conversions.falconCountsToMeters(mBackLeft.getSelectedSensorVelocity() * 10, DriveConstants.kGearing, DriveConstants.kWheelDiameter),
      Conversions.falconCountsToMeters(mBackRight.getSelectedSensorVelocity() * 10, DriveConstants.kGearing, DriveConstants.kWheelDiameter)
    );
  }

  //distances measured in meters
  public MecanumDriveWheelPositions getCurrentDistances() {
    return new MecanumDriveWheelPositions(
      Conversions.falconCountsToMeters(mFrontLeft.getSelectedSensorPosition(), DriveConstants.kGearing, DriveConstants.kWheelDiameter),
      Conversions.falconCountsToMeters(mFrontRight.getSelectedSensorPosition(), DriveConstants.kGearing, DriveConstants.kWheelDiameter),
      Conversions.falconCountsToMeters(mBackLeft.getSelectedSensorPosition(), DriveConstants.kGearing, DriveConstants.kWheelDiameter),
      Conversions.falconCountsToMeters(mBackRight.getSelectedSensorPosition(), DriveConstants.kGearing, DriveConstants.kWheelDiameter)
    );
  }

  public double[] getMotorSets() {
    return new double[]{
      mFrontLeft.get(),
      mFrontRight.get(),
      mBackLeft.get(),
      mBackRight.get()
    };
  }

  // what we want
  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {

    final double frontLeftFeedForward = mFeedForward.calculate(speeds.frontLeftMetersPerSecond);
    final double frontRightFeedForward = mFeedForward.calculate(speeds.frontRightMetersPerSecond);
    final double backLeftFeedForward = mFeedForward.calculate(speeds.rearLeftMetersPerSecond);
    final double backRightFeedForward = mFeedForward.calculate(speeds.rearRightMetersPerSecond);

    final double frontLeftOutput = mFrontLeftPIDController.calculate(
      getCurrentState().frontLeftMetersPerSecond,
      speeds.frontLeftMetersPerSecond
    );
    
    final double frontRightOutput = mFrontRightPIDController.calculate(
      getCurrentState().frontRightMetersPerSecond,
      speeds.frontRightMetersPerSecond
    );

    final double backLeftOutput = mBackLeftPIDController.calculate(
      getCurrentState().rearLeftMetersPerSecond,
      speeds.rearLeftMetersPerSecond
    );

    final double backRightOutput = mBackRightPIDController.calculate(
      getCurrentState().rearRightMetersPerSecond,
      speeds.rearRightMetersPerSecond
    );

    Logger.getInstance().recordOutput("Drivetrain/Front Left Velocity Setpoint", speeds.frontLeftMetersPerSecond);
    Logger.getInstance().recordOutput("Drivetrain/Front Right Velocity Setpoint", speeds.frontRightMetersPerSecond);
    Logger.getInstance().recordOutput("Drivetrain/Back Left Velocity Setpoint", speeds.rearLeftMetersPerSecond);
    Logger.getInstance().recordOutput("Drivetrain/Back Right Velocity Setpoint", speeds.rearRightMetersPerSecond);

    mFrontLeft.set(MathUtil.clamp(frontLeftFeedForward + frontLeftOutput, -12, 12) / 12d);
    mFrontRight.set(MathUtil.clamp(frontRightFeedForward + frontRightOutput, -12, 12) / 12d);
    mBackLeft.set(MathUtil.clamp(backLeftFeedForward + backLeftOutput, -12, 12) / 12d);
    mBackRight.set(MathUtil.clamp(backRightFeedForward + backRightOutput, -12, 12) / 12d);

  }

  public Pose2d getPose() {
    return mPoseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d position) {
    mPoseEstimator.resetPosition(mPigeon.getRotation2d(), getCurrentDistances(), position);
  }
  
  public void stop() {
    mFrontRight.stopMotor();
    mFrontLeft.stopMotor();
    mBackRight.stopMotor();
    mBackLeft.stopMotor();
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var mecanumDriveWheelSpeeds =
        mKinematics.toWheelSpeeds(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, mPigeon.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    setSpeeds(mecanumDriveWheelSpeeds);

  }

  public void updateOdometry() {
    mPoseEstimator.update(mPigeon.getRotation2d(), getCurrentDistances());

    Optional<EstimatedRobotPose> result = mVision.getEstimatedGlobalPose(mPoseEstimator.getEstimatedPosition());

    if (result.isPresent()) {
      EstimatedRobotPose camPose = result.get();
      Logger.getInstance().recordOutput("Drivetrain/Vision Pose", camPose.estimatedPose.toPose2d());
      mPoseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
    }

    Logger.getInstance().recordOutput("Drivetrain/Odometry Pose", mPoseEstimator.getEstimatedPosition());

  }
 
  @Override
  public void simulationPeriodic() {
      mSimulation.update();
  }

  public MecanumDriveKinematics getKinematics() {
    return mKinematics;
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              this.resetPose(traj.getInitialHolonomicPose());
          }
        }),
        new PPMecanumControllerCommand(
            traj, 
            this::getPose, // Pose supplier
            this.mKinematics, // MecanumDriveKinematics
            DriveConstants.kTrajectoryPID, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            DriveConstants.kTrajectoryPID, // Y controller (usually the same values as X controller)
            DriveConstants.kTrajectoryPID, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            DriveConstants.kTrajectoryMaxSpeed, // Max wheel velocity meters per second
            this::setSpeeds, // MecanumDriveWheelSpeeds consumer
            false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
        )
    );
  }

  public Command DriverControl(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot, boolean fieldRelative) {
    return new RunCommand(() -> {
      drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rot.getAsDouble(), fieldRelative);
    }, 
      this
    );
    
  }

}

 