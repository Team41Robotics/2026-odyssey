package frc.robot.Util;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class InterpTables {
  public InterpolatingTreeMap<Double, Double> flywheelSpeed = 
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
 
  public InterpTables() {
    // Example data points, replace with actual values
    flywheelSpeed.put(1.0, 10.0); // Distance (m) to Flywheel Speed (RPS)
    flywheelSpeed.put(2.0, 20.0);
    flywheelSpeed.put(10.0, 100.0);
  }
}