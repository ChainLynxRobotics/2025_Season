package frc.robot.util;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public final class RobotMath {
  /**
   * @param measure measure to check.
   * @param bounds bounds to check against.
   * @return true if -bounds < measure < bounds, false otherwise.
   */
  public static <U extends Unit> boolean measureWithinBounds(
      Measure<U> measure, Measure<U> bounds) {
    return measure.lt(bounds) && measure.gt(bounds.unaryMinus());
  }

  /**
   * @param measure measure to check.
   * @param lower lower bound to check against.
   * @param upper upper bound to check against.
   * @return true if lower < measure < upper, false otherwise.
   */
  public static <U extends Unit> boolean measureWithinBounds(
      Measure<U> measure, Measure<U> lower, Measure<U> upper) {
    return measure.lt(upper) && measure.gt(lower);
  }

  /**
   * @param measure measure to get the absolute value of.
   * @param other measuere to get dist to.
   * @return abs of measuere1 - measuere2.
   */
  public static <U extends Unit, M extends Measure<U>> M dist(M measure1, M measure2) {
    return abs((M) measure1.minus(measure2));
  }

  /**
   * @param measure measure to get the absolute value of.
   * @return the absulute value of measure with the zero point of the base unit for that type.
   */
  public static <U extends Unit, M extends Measure<U>> M abs(M measure) {
    return measure.gt((M) measure.baseUnit().zero()) ? measure : (M) measure.unaryMinus();
  }
}
