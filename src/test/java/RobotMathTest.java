import static edu.wpi.first.units.Units.*;
import static frc.robot.util.RobotMath.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Per;
import org.junit.jupiter.api.Test;

class RobotMathTest {
  @Test
  void testMeasureWithinBoundsAbs() {
    assertTrue(measureWithinBounds(Meters.of(0), Meters.of(1)));

    // out of bounds
    assertFalse(measureWithinBounds(Meters.of(2), Meters.of(1)));
    assertFalse(measureWithinBounds(Meters.of(10), Meters.of(0)));

    // Test when measurement equals bounds
    assertFalse(measureWithinBounds(Meters.of(1), Meters.of(1)));
    assertFalse(measureWithinBounds(Meters.of(-1), Meters.of(1)));

    // Test NaN
    assertFalse(measureWithinBounds(Meters.of(Double.NaN), Meters.of(1)));
    assertFalse(measureWithinBounds(Meters.of(0), Meters.of(Double.NaN)));
    assertFalse(measureWithinBounds(Meters.of(Double.NaN), Meters.of(Double.NaN)));
  }

  /**
   * Tests the measureWithinBounds method to ensure it correctly identifies whether a given
   * measurement is within specified bounds.
   */
  @Test
  void testMeasureWithinBoundsLtGt() {

    // Test cases where the measurement is within the bounds
    assertTrue(measureWithinBounds(Meters.of(0), Meters.of(-1), Meters.of(1)));
    // Make sure it still works with all negative values
    assertTrue(measureWithinBounds(Meters.of(-5), Meters.of(-10), Meters.of(-1)));

    // Test cases where the measurement is outside the bounds

    // Make sure it returns false when the measurement is equal to one or both of the bounds
    assertFalse(measureWithinBounds(Meters.of(-1), Meters.of(-1), Meters.of(-1)));
    assertFalse(measureWithinBounds(Meters.of(0), Meters.of(0), Meters.of(1)));

    assertFalse(measureWithinBounds(Meters.of(2), Meters.of(-1), Meters.of(1)));

    // Use non integer numbers
    assertFalse(measureWithinBounds(Meters.of(-2), Meters.of(-1.5), Meters.of(-1)));

    // Swap bounds
    assertFalse(measureWithinBounds(Meters.of(0), Meters.of(1), Meters.of(-1)));
    assertFalse(measureWithinBounds(Meters.of(0), Meters.of(3), Meters.of(2)));
    assertFalse(measureWithinBounds(Meters.of(0), Meters.of(-1), Meters.of(-2)));

    // Outside of bounds
    assertFalse(measureWithinBounds(Meters.of(5), Meters.of(-1), Meters.of(1)));

    // NaN
    assertFalse(measureWithinBounds(Meters.of(Double.NaN), Meters.of(-1), Meters.of(1)));
    assertFalse(measureWithinBounds(Meters.of(0), Meters.of(-1), Meters.of(Double.NaN)));
    assertFalse(
        measureWithinBounds(Meters.of(Double.NaN), Meters.of(Double.NaN), Meters.of(Double.NaN)));
  }

  @Test
  void testDist() {
    assertEquals(Meters.of(3), dist(Meters.of(1), Meters.of(4)));
    assertEquals(Meters.of(3), dist(Meters.of(4), Meters.of(1)));
    assertEquals(Meters.of(0), dist(Meters.of(-5), Meters.of(-5)));
    assertTrue(isMeasureNaN(dist(Meters.of(Double.NaN), Meters.of(1))));
    assertTrue(isMeasureNaN(dist(Meters.of(1), Meters.of(Double.NaN))));
    assertTrue(isMeasureNaN(dist(Meters.of(Double.NaN), Meters.of(Double.NaN))));
    // same testing, but with inches
    assertEquals(Inches.of(3), dist(Inches.of(1), Inches.of(4)));
    assertEquals(Inches.of(3), dist(Inches.of(4), Inches.of(1)));
    assertEquals(Inches.of(0), dist(Inches.of(-5), Inches.of(-5)));
    assertTrue(isMeasureNaN(dist(Inches.of(Double.NaN), Inches.of(1))));
    assertTrue(isMeasureNaN(dist(Inches.of(1), Inches.of(Double.NaN))));
    assertTrue(isMeasureNaN(dist(Inches.of(Double.NaN), Inches.of(Double.NaN))));
    // testing mixed units
    assertEquals(Inches.of(11), dist(Feet.of(1), Inches.of(1)));
  }

  @Test
  void testAbs() {
    assertTrue(abs(Meters.zero()).compareTo(Meters.zero()) == 0);
    assertTrue(abs(Meters.of(1)).in(Meters) == 1);
    assertTrue(abs(Meters.of(-1)).in(Meters) == 1);
    assertTrue(abs(Meters.of(Double.POSITIVE_INFINITY)).in(Meters) == Double.POSITIVE_INFINITY);
    assertTrue(abs(Meters.of(Double.NEGATIVE_INFINITY)).in(Meters) == Double.POSITIVE_INFINITY);
    assertTrue(Double.isNaN(abs(Meters.of(Double.NaN)).in(Meters)));
    assertTrue(Double.isNaN(abs(Meters.of(-Double.NaN)).in(Meters)));
    assertTrue(abs(Meters.of(1e-7)).in(Meters) == 1e-7);
    assertTrue(abs(Meters.of(-1e-7)).in(Meters) == 1e-7);
    assertTrue(abs(Meters.of(1e+6)).in(Meters) == 1e+6);
    assertTrue(abs(Meters.of(-1e+6)).in(Meters) == 1e+6);
  }

  @Test
  void testIsWithinTolerance() {
    assertTrue(isWithinTolerance(Meters.zero(), Meters.zero(), Meters.zero()));
    assertFalse(isWithinTolerance(Meters.zero(), Meters.of(1e-6), Meters.zero()));
    assertTrue(isWithinTolerance(Meters.zero(), Meters.zero(), Meters.of(1)));
    assertTrue(
        isWithinTolerance(Meters.zero(), Meters.zero(), Meters.of(Double.POSITIVE_INFINITY)));
    assertFalse(isWithinTolerance(Meters.zero(), Meters.of(1), Meters.zero()));
    assertTrue(isWithinTolerance(Meters.zero(), Meters.of(1), Meters.of(1)));
    assertTrue(isWithinTolerance(Meters.zero(), Meters.of(1), Meters.of(Double.POSITIVE_INFINITY)));
    assertFalse(isWithinTolerance(Meters.of(1), Meters.zero(), Meters.zero()));
    assertTrue(isWithinTolerance(Meters.of(1), Meters.zero(), Meters.of(1)));
    assertTrue(isWithinTolerance(Meters.of(1), Meters.zero(), Meters.of(Double.POSITIVE_INFINITY)));
    assertTrue(isWithinTolerance(Meters.of(1), Meters.of(1), Meters.zero()));
    assertFalse(isWithinTolerance(Meters.of(Double.NaN), Meters.zero(), Meters.zero()));
    assertFalse(isWithinTolerance(Meters.zero(), Meters.of(Double.NaN), Meters.zero()));
    assertFalse(isWithinTolerance(Meters.zero(), Meters.zero(), Meters.of(Double.NaN)));
    assertFalse(
        isWithinTolerance(Meters.of(Double.NaN), Meters.of(Double.NaN), Meters.of(Double.NaN)));
  }

  @Test
  void testApproxZero() {
    assertTrue(approxZero(Meters.of(0)));
    assertFalse(approxZero(Meters.of(1)));
    assertFalse(approxZero(Meters.of(-1)));
    assertFalse(approxZero(Meters.of(Double.POSITIVE_INFINITY)));
    assertFalse(approxZero(Meters.of(Double.NEGATIVE_INFINITY)));
    assertTrue(approxZero(Meters.of(1e-6)));
    assertTrue(approxZero(Meters.of(-1e-6)));
    assertFalse(approxZero(Meters.of(2e-6)));
    assertFalse(approxZero(Meters.of(-2e-6)));
    assertTrue(approxZero(Meters.of(1e-7)));
    assertTrue(approxZero(Meters.of(-1e-7)));
    assertTrue(approxZero(Meters.of(2e-7)));
    assertTrue(approxZero(Meters.of(-2e-7)));
    assertFalse(approxZero(Meters.of(Double.NaN)));
    assertFalse(approxZero(Meters.of(-Double.NaN)));
    assertTrue(approxZero(Inches.of(1e-8)));
    assertTrue(approxZero(Inches.of(-1e-8)));
    assertTrue(approxZero(Inches.of(0)));
    assertFalse(approxZero(Inches.of(Double.POSITIVE_INFINITY)));
    assertFalse(approxZero(Inches.of(Double.NEGATIVE_INFINITY)));
    assertTrue(approxZero(Celsius.of(0)));
    assertFalse(approxZero(Celsius.of(1)));
    // WPILib subtracting temps are weird
    // assertFalse(approxZero(Celsius.of(Kelvin.zero().in(Celsius))));
  }

  @Test
  void testApproxZeroTol() {
    assertTrue(approxZero(Meters.of(0), Meters.of(1)));
    assertTrue(approxZero(Meters.of(1), Meters.of(1)));
    assertTrue(approxZero(Meters.of(-1), Meters.of(1)));
    assertFalse(approxZero(Meters.of(Double.POSITIVE_INFINITY), Meters.of(1)));
    assertFalse(approxZero(Meters.of(Double.NEGATIVE_INFINITY), Meters.of(1)));
    assertTrue(approxZero(Meters.of(1e-6), Meters.of(1)));
    assertTrue(approxZero(Meters.of(-1e-6), Meters.of(1)));
    assertTrue(approxZero(Meters.of(2e-6), Meters.of(1)));
    assertTrue(approxZero(Meters.of(-2e-6), Meters.of(1)));
    assertFalse(approxZero(Meters.of(Double.NaN), Meters.of(1)));
    assertFalse(approxZero(Meters.of(-Double.NaN), Meters.of(1)));
    assertTrue(approxZero(Inches.of(1e-8), Meters.of(1)));
    assertTrue(approxZero(Inches.of(-1e-8), Meters.of(1)));
    assertTrue(approxZero(Inches.of(0), Meters.of(1)));
    assertFalse(approxZero(Inches.of(Double.POSITIVE_INFINITY), Meters.of(1)));
    assertFalse(approxZero(Inches.of(Double.NEGATIVE_INFINITY), Meters.of(1)));
    // WPILib subtracting temps are weird
    // assertTrue(approxZero(Celsius.of(0), Kelvin.of(1)));
    // assertTrue(approxZero(Celsius.of(1), Kelvin.of(1)));
    assertFalse(approxZero(Celsius.of(Kelvin.zero().in(Celsius)), Kelvin.of(1)));
    assertTrue(approxZero(Celsius.of(0), Celsius.of(1)));
    assertTrue(approxZero(Celsius.of(1), Celsius.of(1)));
    // WPILib subtracting temps are weird
    // assertFalse(approxZero(Celsius.of(Kelvin.zero().in(Celsius)), Celsius.of(1)));
  }

  @Test
  void testIsMeasureNaN() {
    assertTrue(isMeasureNaN(Meters.of(Double.NaN)));
    assertTrue(isMeasureNaN(Inches.of(Double.NaN)));
    assertFalse(isMeasureNaN(Meters.of(0)));
    assertFalse(isMeasureNaN(Inches.of(Double.POSITIVE_INFINITY)));
    assertFalse(isMeasureNaN(Inches.of(Double.NEGATIVE_INFINITY)));
  }

  @Test
  void testDistanceBetweenTranslations() {
    assertEquals(
        distanceBetweenTranslations(new Translation3d(), new Translation3d()), Meters.zero());
    assertEquals(
        distanceBetweenTranslations(new Translation3d(), new Translation3d(1, 0, 0)), Meters.of(1));
    assertEquals(
        distanceBetweenTranslations(new Translation3d(), new Translation3d(0, 1, 0)), Meters.of(1));
    assertEquals(
        distanceBetweenTranslations(new Translation3d(), new Translation3d(0, 0, 1)), Meters.of(1));
    assertEquals(
        distanceBetweenTranslations(new Translation3d(1, 0, 0), new Translation3d()), Meters.of(1));
    assertEquals(
        distanceBetweenTranslations(new Translation3d(0, 1, 0), new Translation3d()), Meters.of(1));
    assertEquals(
        distanceBetweenTranslations(new Translation3d(0, 0, 1), new Translation3d()), Meters.of(1));
    assertEquals(
        distanceBetweenTranslations(new Translation3d(), new Translation3d(1, 1, 0)),
        Meters.of(Math.sqrt(2)));
    assertEquals(
        distanceBetweenTranslations(new Translation3d(), new Translation3d(0, 1, 1)),
        Meters.of(Math.sqrt(2)));
    assertEquals(
        distanceBetweenTranslations(new Translation3d(), new Translation3d(1, 0, 1)),
        Meters.of(Math.sqrt(2)));
    assertEquals(
        distanceBetweenTranslations(new Translation3d(1, 1, 0), new Translation3d()),
        Meters.of(Math.sqrt(2)));
    assertEquals(
        distanceBetweenTranslations(new Translation3d(0, 1, 1), new Translation3d()),
        Meters.of(Math.sqrt(2)));
    assertEquals(
        distanceBetweenTranslations(new Translation3d(1, 0, 1), new Translation3d()),
        Meters.of(Math.sqrt(2)));
    assertEquals(
        distanceBetweenTranslations(new Translation3d(), new Translation3d(1, 1, 1)),
        Meters.of(Math.sqrt(3)));
    assertEquals(
        distanceBetweenTranslations(new Translation3d(1, 1, 1), new Translation3d()),
        Meters.of(Math.sqrt(3)));
    assertTrue(
        Double.isInfinite(
            distanceBetweenTranslations(
                    new Translation3d(), new Translation3d(Double.POSITIVE_INFINITY, 0, 0))
                .in(Meters)));
    assertTrue(
        Double.isInfinite(
            distanceBetweenTranslations(
                    new Translation3d(), new Translation3d(0, Double.POSITIVE_INFINITY, 0))
                .in(Meters)));
    assertTrue(
        Double.isInfinite(
            distanceBetweenTranslations(
                    new Translation3d(), new Translation3d(0, 0, Double.POSITIVE_INFINITY))
                .in(Meters)));
    assertTrue(
        Double.isInfinite(
            distanceBetweenTranslations(
                    new Translation3d(Double.POSITIVE_INFINITY, 0, 0), new Translation3d())
                .in(Meters)));
    assertTrue(
        Double.isInfinite(
            distanceBetweenTranslations(
                    new Translation3d(0, Double.POSITIVE_INFINITY, 0), new Translation3d())
                .in(Meters)));
    assertTrue(
        Double.isInfinite(
            distanceBetweenTranslations(
                    new Translation3d(0, 0, Double.POSITIVE_INFINITY), new Translation3d())
                .in(Meters)));

    assertTrue(
        Double.isNaN(
            distanceBetweenTranslations(new Translation3d(Double.NaN, 0, 0), new Translation3d())
                .in(Meters)));
    assertTrue(
        Double.isNaN(
            distanceBetweenTranslations(new Translation3d(0, Double.NaN, 0), new Translation3d())
                .in(Meters)));
    assertTrue(
        Double.isNaN(
            distanceBetweenTranslations(new Translation3d(0, 0, Double.NaN), new Translation3d())
                .in(Meters)));
    assertTrue(
        Double.isNaN(
            distanceBetweenTranslations(new Translation3d(), new Translation3d(Double.NaN, 0, 0))
                .in(Meters)));
    assertTrue(
        Double.isNaN(
            distanceBetweenTranslations(new Translation3d(), new Translation3d(0, Double.NaN, 0))
                .in(Meters)));
    assertTrue(
        Double.isNaN(
            distanceBetweenTranslations(new Translation3d(), new Translation3d(0, 0, Double.NaN))
                .in(Meters)));
  }

  @Test
  void testUseConvertionFactorFromLowerOrderUnitForHigherOrderConversion() {
    Per<DistanceUnit, AngleUnit> conversionFactor1 = Meters.one().div(Radians.one());

    assertTrue(
        castToMoreSpecificUnits(
                applyDimesionalAnalysis(conversionFactor1, RadiansPerSecond.zero()),
                MetersPerSecond.zero())
            .isEquivalent(MetersPerSecond.zero()));
    assertTrue(
        castToMoreSpecificUnits(
                applyDimesionalAnalysis(conversionFactor1, RadiansPerSecond.one()),
                MetersPerSecond.zero())
            .isEquivalent(MetersPerSecond.one()));
    assertTrue(
        castToMoreSpecificUnits(
                applyDimesionalAnalysis(conversionFactor1, RadiansPerSecond.of(2)),
                MetersPerSecond.zero())
            .isEquivalent(MetersPerSecond.of(2)));
    assertTrue(
        castToMoreSpecificUnits(
                applyDimesionalAnalysis(conversionFactor1, RadiansPerSecond.of(3)),
                MetersPerSecond.zero())
            .isEquivalent(MetersPerSecond.of(3)));
    System.out.println(
        castToMoreSpecificUnits(
            applyDimesionalAnalysis(
                conversionFactor1, RadiansPerSecond.of(Double.POSITIVE_INFINITY)),
            MetersPerSecond.zero()));
    System.out.println(MetersPerSecond.of(Double.POSITIVE_INFINITY));
    assertTrue(
        Double.isInfinite(
            castToMoreSpecificUnits(
                    applyDimesionalAnalysis(
                        conversionFactor1, RadiansPerSecond.of(Double.POSITIVE_INFINITY)),
                    MetersPerSecond.zero())
                .in(FeetPerSecond)));
    assertTrue(
        Double.isInfinite(
            castToMoreSpecificUnits(
                    applyDimesionalAnalysis(
                        conversionFactor1, RadiansPerSecond.of(Double.NEGATIVE_INFINITY)),
                    MetersPerSecond.zero())
                .in(FeetPerSecond)));
    assertTrue(
        isMeasureNaN(
            castToMoreSpecificUnits(
                applyDimesionalAnalysis(conversionFactor1, RadiansPerSecond.of(Double.NaN)),
                MetersPerSecond.zero())));
    assertTrue(
        castToMoreSpecificUnits(
                applyDimesionalAnalysis(conversionFactor1, RadiansPerSecond.of(-1)),
                MetersPerSecond.zero())
            .isEquivalent(MetersPerSecond.of(-1)));
    assertTrue(
        castToMoreSpecificUnits(
                applyDimesionalAnalysis(conversionFactor1, RadiansPerSecond.of(-2)),
                MetersPerSecond.zero())
            .isEquivalent(MetersPerSecond.of(-2)));
    assertTrue(
        isMeasureNaN(
            castToMoreSpecificUnits(
                applyDimesionalAnalysis(conversionFactor1, RadiansPerSecond.of(-Double.NaN)),
                MetersPerSecond.zero())));
  }

  @Test
  void testCastToMoreSpecificUnits() {
    for (int i = -0xFF; i <= 0xFF; i++) {
      for (int j = Integer.MIN_VALUE / 0xFFFF; j <= Integer.MAX_VALUE / 0xFFFF; j++) {
        assertTrue(
            castToMoreSpecificUnits(MetersPerSecond.of(j), MetersPerSecond.of(i))
                .isEquivalent(MetersPerSecond.of(j)));
      }
      assertTrue(
          Double.isInfinite(
              castToMoreSpecificUnits(
                      MetersPerSecond.of(Double.POSITIVE_INFINITY), MetersPerSecond.of(i))
                  .in(MetersPerSecond)));
      assertTrue(
          Double.isInfinite(
              castToMoreSpecificUnits(
                      MetersPerSecond.of(Double.NEGATIVE_INFINITY), MetersPerSecond.of(i))
                  .in(MetersPerSecond)));
      assertTrue(
          Double.isNaN(
              castToMoreSpecificUnits(MetersPerSecond.of(Double.NaN), MetersPerSecond.of(i))
                  .in(MetersPerSecond)));
    }
    for (int j = Integer.MIN_VALUE / 0xFFFF; j <= Integer.MAX_VALUE / 0xFFFF; j++) {
      assertTrue(
          castToMoreSpecificUnits(MetersPerSecond.of(j), MetersPerSecond.of(Double.NaN))
              .isEquivalent(MetersPerSecond.of(j)));
    }
    assertTrue(
        Double.isInfinite(
            castToMoreSpecificUnits(
                    MetersPerSecond.of(Double.POSITIVE_INFINITY), MetersPerSecond.of(Double.NaN))
                .in(MetersPerSecond)));
    assertTrue(
        Double.isInfinite(
            castToMoreSpecificUnits(
                    MetersPerSecond.of(Double.NEGATIVE_INFINITY), MetersPerSecond.of(Double.NaN))
                .in(MetersPerSecond)));
    assertTrue(
        Double.isNaN(
            castToMoreSpecificUnits(MetersPerSecond.of(Double.NaN), MetersPerSecond.of(Double.NaN))
                .in(MetersPerSecond)));
  }

  @Test
  void testSignBool() {
    assertEquals(signBool(false), -1);
    assertEquals(signBool(true), 1);
  }
}
