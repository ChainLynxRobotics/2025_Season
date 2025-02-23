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
    if (Double.isNaN(measure.baseUnitMagnitude())) {
      return false;
    }
    if (Double.isNaN(bounds.baseUnitMagnitude())) {
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
    if (Double.isNaN(measure.baseUnitMagnitude())) {
      return false;
    }
    if (Double.isNaN(lower.baseUnitMagnitude())) {
      return false;
    }
    if (Double.isNaN(upper.baseUnitMagnitude())) {
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
    if (Double.isNaN(measure1.baseUnitMagnitude())) {
      // NaN's propagate
      return (M) measure1.baseUnit().of(Double.NaN);
    }
    return abs((M) measure1.minus(measure2));
  }

  /**
   * @param measure measure to get the absolute value of.
   * @return the absulute value of measure with the zero point of the unit for that type.
   */
  @SuppressWarnings("unchecked")
  public static <U extends Unit, M extends Measure<U>> M abs(M measure) {
    if (Double.isNaN(measure.baseUnitMagnitude())) {
      // NaN's propagate
      return (M) measure.baseUnit().of(Double.NaN);
    }
    return measure.gte((M) measure.unit().zero())
        ? measure
        : (M) ((M) measure.unit().zero()).minus(measure);
  }

  public static <U extends Unit> boolean isWithinTolerance(
      Measure<U> measure1, Measure<U> measure2, Measure<U> tol) {
    if (Double.isNaN(measure1.baseUnitMagnitude())) {
      return false;
    }
    if (Double.isNaN(measure2.baseUnitMagnitude())) {
      return false;
    }
    if (Double.isNaN(tol.baseUnitMagnitude())) {
      return false;
    }
    return dist(measure1, measure2).lte(tol);
  }

  @SuppressWarnings("unchecked")
  public static <U extends Unit> boolean approxZero(Measure<U> measure) {
    return approxZero(measure, (Measure<U>) measure.unit().of(1e-6));
  }

  public static <U extends Unit> boolean approxZero(Measure<U> measure, Measure<U> tol) {
    if (Double.isNaN(measure.in(measure.unit()))) {
      return false;
    }
    return abs(measure).lte(tol);
  }

  public static <U extends Unit> boolean isMeasureNaN(Measure<U> measure) {
    return Double.isNaN(measure.baseUnitMagnitude());
  }
}
