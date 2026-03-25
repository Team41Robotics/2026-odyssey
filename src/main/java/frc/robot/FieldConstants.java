package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;



public class FieldConstants {
private static final double HEX_OFFSET_METERS = 60.0 * 0.0254;

// Field-relative positions
public static final Translation2d redAllianceHub = new Translation2d(-HEX_OFFSET_METERS, 0.0);
public static final Translation2d blueAllianceHub = new Translation2d(HEX_OFFSET_METERS, 0.0);
}