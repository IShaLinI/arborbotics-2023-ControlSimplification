package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;

public final class Constants {

  public static class RobotConstants {
    public static final int kMaximumVoltage = 10;
  }

  public static class CAN {

    public static final int kFrontLeft = 1;
    public static final int kFrontRight = 2;
    public static final int kBackRight = 3;
    public static final int kBackLeft = 4;
    public static final int kPivot = 5;
    public static final int kPigeon = 6;
    public static final int kPCM = 7;
    public static final int kLeftClaw = 8;
    public static final int kRightClaw = 9;
    public static final int kArm = 13;

  }

  public static class ClawConstants {
    public static final int kLeftPistonForward = 12;
    public static final int kLeftPistonReverse = 13;

    public static final int kRightPistonForward = 8;
    public static final int kRightPistonReverse = 9;

    public static final SupplyCurrentLimitConfiguration kCurrentLimit = new SupplyCurrentLimitConfiguration(true, 10, 10, 0);

    public static final double kInSpeed = 0.6;
    public static final double kOutSpeed = 0.4;

    public static enum State {
      GRAB(Value.kForward, kInSpeed),
      RELEASE(Value.kForward, kOutSpeed),
      NEUTRAL(Value.kReverse, 0),
      START(Value.kForward, 0);

      public final Value value;
      public final double speed;

      State(Value value, double speed) {
        this.value = value;
        this.speed = speed;
      }

    }

  }

  public static class DriveConstants {

    //Robot Characteristics
    public static final double kWheelDiameter = Units.inchesToMeters(6);
    public static final double kGearing = 1 / KitbotGearing.k10p71.value;
    public static final double kFalconToMeters = (1.0 / Conversions.FALCON_CPR) * (kWheelDiameter * Math.PI) * kGearing;
    
    //Characterization Values
    public static final double kS = 0.13305;
    public static final double kV = 2.2876;
    public static final double kA = 0.31596;

    public static double[] kDrivetrainCharacterization = {kS, kV, kA};

    public static final SupplyCurrentLimitConfiguration kCurrentLimit = new SupplyCurrentLimitConfiguration(true, 25, 25, 0);

    public static final Translation2d kWheelPositions[] = {
      new Translation2d(0.291841, 0.258571),
      new Translation2d(0.291841, -0.258571),
      new Translation2d(-0.291841, 0.258571),
      new Translation2d(-0.291841, -0.258571)
    };

    public static final PIDController kWheelPID = new PIDController(0.4, 0, 0);
    public static final PIDConstants kTrajTranslationPID = new PIDConstants(6, 0, 0);
    public static final PIDConstants kTrajRotationPID = new PIDConstants(3, 0, 0);
    
    //Meters/s
    public static final double kTrajectoryMaxSpeed = 3;
    public static final double kTrajectoryMaxAccel = 6;
    public static final double kMaxTranslationSpeed = 3;
    //Rad/s
    public static final double kMaxRotationSpeed = Math.PI;

    //Meters/s/s
    public static final SlewRateLimiter kXTranslationLimiter = new SlewRateLimiter(2 * kMaxTranslationSpeed,2 * -kMaxTranslationSpeed, 0);
    public static final SlewRateLimiter kYTranslationLimiter = new SlewRateLimiter(2 * kMaxTranslationSpeed,2 * -kMaxTranslationSpeed, 0);
    
    //Rad/s/s
    public static final SlewRateLimiter kRotationLimiter = new SlewRateLimiter(2 * kMaxRotationSpeed, 2 * -kMaxRotationSpeed, 0);

  }

  public static class ExtentionConstants{
    public static final double kGearing = 1d / 16d;
    public static final double kSpoolDiameter = Units.inchesToMeters(0.95);

    public static final PIDController kPID = new PIDController(20/(Units.inchesToMeters(37)), 0, 0);
    public static final SupplyCurrentLimitConfiguration kCurrentLimit = new SupplyCurrentLimitConfiguration(true, 10, 10, 0);
  
    //Cad Numbers
    public static final double kMinDistance = Units.inchesToMeters(41.205931);
    public static final double kMaxDistance = Units.inchesToMeters(66.324751);

    public static enum SETPOINTS {

      //TODO figure these out
      INTAKE(0), 
      CARRY(0), 
      SCORE(0),
      SUBSTATION(0),
      START(0);

      public final double distance;

      private SETPOINTS(double distance) {
        this.distance = distance;
      }
    }
  
  }

  public static class PivotConstants {  
    
    public static final double kGearing = (1d / 100d) * (16d / 60d);
    public static final int kThroughboreDIO = 9;
    public static final double kThroughboreOffset = 0.6789;
    public static final PIDController kPID = new PIDController(2d/3d, 0, 0);
    public static final double kPositionTollerance = 1; //1 Degree
    public static final double kVelocotiyTollerance = 5; //5 Degrees/s

    public static final SupplyCurrentLimitConfiguration kCurrentLimit = new SupplyCurrentLimitConfiguration(true, 25, 25, 0);

    public static final int kRatchetForward = 6;
    public static final int kRatchetReverse = 7;

    public static final double kMaxAngle = 80;
    public static final double kMinAngle = -20;

    public static enum SETPOINTS {

      INTAKE(-12), 
      ZERO(0),
      CARRY(30), 
      SCORE(50),
      SUBSTATION(55),
      START(66);

      public final int angle;

      private SETPOINTS(int angle) {
        this.angle = angle;
      }
    }
  }
  
  public static class VisionConstants {
    public static final Transform3d kRobotToCam = new Transform3d(
      new Translation3d(0.043, -0.203, 0.589065),
      new Rotation3d()
    );
  }
}
