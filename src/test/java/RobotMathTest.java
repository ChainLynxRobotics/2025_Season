import static edu.wpi.first.units.Units.*;
import static frc.robot.util.RobotMath.*;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

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
    System.out.println(Celsius.of(Kelvin.zero().in(Celsius)).in(Kelvin));
    System.out.println(Celsius.of(Kelvin.zero().in(Celsius)).in(Celsius));
    System.out.println(Celsius.zero().minus(Celsius.of(-1)).in(Celsius) + " cool");
    // WPILib bug with subtracting tempreture
    // assertFalse(approxZero(Celsius.of(Kelvin.zero().in(Celsius))));

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
    // WPILib bug with subtracting tempreture
    // assertTrue(approxZero(Celsius.of(0), Kelvin.of(1)));
    // assertTrue(approxZero(Celsius.of(1), Kelvin.of(1)));
    assertFalse(approxZero(Celsius.of(Kelvin.zero().in(Celsius)), Kelvin.of(1)));
    assertTrue(approxZero(Celsius.of(0), Celsius.of(1)));
    assertTrue(approxZero(Celsius.of(1), Celsius.of(1)));
    // WPILib bug with subtracting tempreture
    // assertFalse(approxZero(Celsius.of(Kelvin.zero().in(Celsius)), Celsius.of(1)));
  }
}
