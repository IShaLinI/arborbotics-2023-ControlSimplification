package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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

  public static class FalconConstants {

    public static final int CPR = 2048;

    /**
     * @param deg Value in degrees
     * @param gearing Value < 0 for reductions
     * @return Falcon Encoder Counts
     */
    public static int degreesToFalconCounts(double deg, double gearing){
      return (int)(deg * (1d/360d) * (1/gearing) * (1d / CPR));
    }

    public static double falconCountsToDegrees(double counts, double gearing){
      return counts * (1 / CPR) * gearing * 360;
    }

    /**
     * @param counts Falcon encoder counts
     * @param gearing Value < 0 for reductions
     * @param wheelDiameter Diameter of wheel
     * @return meters
     */
    public static double falconCountsToMeters(double counts, double gearing, double wheelDiameter){
      return (1.0 / CPR) * wheelDiameter * Math.PI * gearing;
    }

  }

  public static class ClawConstants {
    public static final int kLeftPistonForward = 12;
    public static final int kLeftPistonReverse = 13;

    public static final int kRightPistonForward = 8;
    public static final int kRightPistonReverse = 9;

    public static final SupplyCurrentLimitConfiguration kCurrentLimit = new SupplyCurrentLimitConfiguration(true, 10, 10, 0);

    public static final double kSpeed = 0.6;

  }

  public static class DriveConstants {

    //ENCODERS YAY FUN 
    public static final double kWheelDiameter = Units.inchesToMeters(6);
    public static final double kGearing = 1 / KitbotGearing.k10p71.value;
    public static final double kFalconToMeters = (1.0 / FalconConstants.CPR) * (kWheelDiameter * Math.PI) * kGearing;
    
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

    public static final PIDController kTrajectoryPID = new PIDController(1, 0, 0);

  }

  public static class VisionConstants {
    
    public static final Transform3d kRobotToCam = new Transform3d(
      new Translation3d(0.043, -0.203, 0.589065),
      new Rotation3d()
    );
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
  public static class ExtentionConstants{
    public static final double kGearing = 1d / 16d;
    public static final double kSpoolDiameter = Units.inchesToMeters(0.95);

    public static final PIDController kPID = new PIDController(20/(Units.inchesToMeters(37)), 0, 0);
    public static final SupplyCurrentLimitConfiguration kCurrentLimit = new SupplyCurrentLimitConfiguration(true, 10, 10, 0);
  
    public static final double kMinDistance = 0;
    public static final double kMaxDistance = Units.inchesToMeters(35);
  
  }
}
