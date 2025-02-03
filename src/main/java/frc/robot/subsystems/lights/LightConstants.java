package frc.robot.subsystems.lights;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Map;

public class LightConstants {
  public static final int pwmPort = 1; // placeholder
  public static final int numLeds = 60;
  public static final Distance LED_SPACING = Meters.of(1.0 / numLeds); // check

  public enum LightsState {
    IDLE,
    HAS_CORAL,
    DRIVING, // based on alliance color
    ELEVATORING,
    INTAKING,
    STOPPED,
    CELEBRATE
  }

  public static Map<LightsState, LEDPattern> ledMap =
      Map.of(
          LightsState.IDLE, LEDPattern.solid(Color.kWhite),
          LightsState.HAS_CORAL, LEDPattern.solid(Color.kOrange),
          LightsState.DRIVING,
              LEDPattern.steps(
                      Map.of(
                          0,
                          Color.kPurple,
                          0.5,
                          DriverStation.getAlliance().get() == Alliance.Blue
                              ? Color.kBlue
                              : Color.kRed))
                  .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.3), LED_SPACING),
          LightsState.ELEVATORING,
              LEDPattern.gradient(GradientType.kContinuous, Color.kRed, Color.kGreen)
                  .mask(
                      LEDPattern.progressMaskLayer(
                          () -> 0.5)), // placeholder that should be elevator.getHeight() /
          // elevator.maxHeight()
          LightsState.INTAKING, LEDPattern.solid(Color.kPurple).breathe(Seconds.of(1)),
          LightsState.STOPPED, LEDPattern.solid(Color.kDarkBlue),
          LightsState.CELEBRATE,
              LEDPattern.rainbow(255, 128)
                  .scrollAtAbsoluteSpeed(MetersPerSecond.of(1), LED_SPACING));
}
