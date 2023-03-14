package frc.robot.util.math;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class FalconUtil {
  
    public static int FALCON_CPR = 2048;
    public static double FALCON_KV = Units.radiansPerSecondToRotationsPerMinute(DCMotor.getFalcon500(1).freeSpeedRadPerSec)/148000; //Dont ask where the 148000 comes from i dont know

    /**
     * @param deg Value in degrees
     * @param gearing Value < 0 for reductions
     * @return Falcon Encoder Counts
     */
    public static int degreesToFalconCounts(double deg, double gearing){
      return (int)(deg * (1d/360d) * (1/gearing) * (1d / FALCON_CPR));
    }

    public static double falconCountsToDegrees(double counts, double gearing){
      return counts * (1 / FALCON_CPR) * gearing * 360;
    }

    public static double falconCountsToRPM(double counts, double gearing){
      return counts * gearing * 600 * (1d/FALCON_CPR);
    }

    /**
     * @param counts Falcon encoder counts
     * @param gearing Value < 0 for reductions
     * @param wheelDiameter Diameter of wheel
     * @return metsers
     */
    public static double falconCountsToMeters(double counts, double gearing, double wheelDiameter){
      return counts * (1.0 / FALCON_CPR) * wheelDiameter * Math.PI * gearing;
    }

  }