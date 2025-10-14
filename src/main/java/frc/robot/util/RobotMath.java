package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;

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
    // No need for NaN checks as minus with return NaN if either are NaN
    return abs((M) measure1.minus(measure2));
  }

  /**
   * @param measure measure to get the absolute value of.
   * @return the absolute value of measure with the zero point of the unit for that type.
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

  public static Distance distanceBetweenTranslations(Translation3d trans1, Translation3d trans2) {
    return Meters.of(trans1.getDistance(trans2));
  }

  public static Distance distanceBetweenTranslations(Translation2d trans1, Translation2d trans2) {
    return distanceBetweenTranslations(new Translation3d(trans1), new Translation3d(trans2));
  }

  public static Distance distanceBetweenPoses(Pose2d pose1, Pose2d pose2) {
    return distanceBetweenTranslations(pose1.getTranslation(), pose2.getTranslation());
  }
  /**
   * Used to convert things like a LinearVelocity into an AngularVelocity using a conversion factor
   * for Distance to Angle
   *
   * @param conversionFactor ConversionFactor in the form Per<U,T>
   * @param measure Measure in the form Measure<PerUnit<T,R>>
   * @return Measure in the form Measure<PerUnit<U,R>>
   */
  @SuppressWarnings("unchecked")
  public static <
          U extends Unit,
          P extends Per<PDividend, U>,
          PDividend extends Unit,
          M extends Measure<MUnit>,
          MUnit extends PerUnit<U, MDivisor>,
          MDivisor extends Unit,
          Out extends Measure<PerUnit<PDividend, MDivisor>>>
      Out applyDimesionalAnalysis(P conversionFactor, M measure) {
    return (Out)
        conversionFactor
            .timesDivisor((Measure<U>) measure.times(measure.unit().denominator().one()))
            .div(measure.unit().denominator().one());
  }

  /**
   * @param measure a Measure in the form Measure<PerUnit<U,T>>
   * @param returnUnit a Measure or a type that extends in the form Measure<SpecificUnit> where
   *     SpecificUnit extends PerUnit<U,T> and has the same base unit. The measure can have any
   *     value;
   * @return Value of the measure passed while wrapped in returnUnit.
   */
  @SuppressWarnings("unchecked")
  public static <
          SpecificUnint extends SuppliedUnit,
          SuppliedUnit extends PerUnit<U, I>,
          U extends Unit,
          I extends Unit,
          M extends Measure<SpecificUnint>,
          T extends Measure<SuppliedUnit>>
      M castToMoreSpecificUnits(T measure, M returnUnit) {

    return (M) returnUnit.unit().ofBaseUnits(measure.baseUnitMagnitude());
  }

  /**
   * @return 1 if b is true, -1 otherwise
   */
  public static int signBool(boolean b) {
    return b ? 1 : -1;
  }
}
