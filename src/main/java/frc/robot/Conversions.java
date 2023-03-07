package frc.robot;

public class Conversions {
  
    public static int FALCON_CPR = 2048;

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