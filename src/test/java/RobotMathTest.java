import static edu.wpi.first.units.Units.*;
import static frc.robot.util.RobotMath.*;

import org.junit.jupiter.api.Assertions.assertFalse;
import org.junit.jupiter.api.Assertions.assertTrue;
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
}
