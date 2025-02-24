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
    if (isMeasureNaN(measure)) {
      return false;
    }
    if (isMeasureNaN(bounds)) {
      return false;
    }
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
    if (isMeasureNaN(measure)) {
      return false;
    }
    if (isMeasureNaN(lower)) {
      return false;
    }
    if (isMeasureNaN(upper)) {
      return false;
    }
    return measure.lt(upper) && measure.gt(lower);
  }

  /**
   * @param measure measure to get the absolute value of.
   * @param other measuere to get dist to.
   * @return abs of measuere1 - measuere2.
   */
  @SuppressWarnings("unchecked")
  public static <U extends Unit, M extends Measure<U>> M dist(M measure1, M measure2) {
    // No need for NaN checks as minus with retrun NaN if eather are NaN
    return abs((M) measure1.minus(measure2));
  }

  /**
   * @param measure measure to get the absolute value of.
   * @return the absulute value of measure with the zero point of the unit for that type.
   */
  @SuppressWarnings("unchecked")
  public static <U extends Unit, M extends Measure<U>> M abs(M measure) {
    if (isMeasureNaN(measure)) {
      // NaN's propagate
      return (M) measure.baseUnit().of(Double.NaN);
    }
    return measure.gte((M) measure.unit().zero())
        ? measure
        : (M) ((M) measure.unit().zero()).minus(measure);
  }

  public static <U extends Unit> boolean isWithinTolerance(
      Measure<U> measure1, Measure<U> measure2, Measure<U> tol) {
    if (isMeasureNaN(measure1)) {
      return false;
    }
    if (isMeasureNaN(measure2)) {
      return false;
    }
    if (isMeasureNaN(tol)) {
      return false;
    }
    return dist(measure1, measure2).lte(tol);
  }

  @SuppressWarnings("unchecked")
  public static <U extends Unit> boolean approxZero(Measure<U> measure) {
    return approxZero(measure, (Measure<U>) measure.unit().of(1e-6));
  }

  public static <U extends Unit> boolean approxZero(Measure<U> measure, Measure<U> tol) {
    if (isMeasureNaN(measure)) {
      return false;
    }
    return abs(measure).lte(tol);
  }

  public static <U extends Unit> boolean isMeasureNaN(Measure<U> measure) {
    return Double.isNaN(measure.baseUnitMagnitude());
  }
}
