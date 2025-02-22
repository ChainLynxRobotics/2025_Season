// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Newtons;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Map;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode currentMode =
      Mode.REAL; // RobotBase.isReal() ? Mode.REAL : Mode.SIM; robot thinks it's fake wut

  // Phisical values of the robot
  public static final double ROBOT_MASS_KG = 74.088;
  public static final double ROBOT_MOI = 6.883;
  public static final double WHEEL_COF = 1.2;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class DriveConstants {
    public static final LinearVelocity maxTranslationSpeed = MetersPerSecond.of(10);
    public static final AngularVelocity maxRotVelocity = RadiansPerSecond.of(4 * Math.PI);
    public static final double DRIVE_DEADBAND = 0.05;
    public static final Distance TAG_DISTANCE = Meters.of(0.3);
    public static final PIDController translationController = new PIDController(0.5, 0.001, 0);
    public static final PIDController rotationController = new PIDController(0.03, 0.01, 0);

    public static final PIDController tipControllerX = new PIDController(0.25, 0, 0.1);
    public static final PIDController tipControllerY = new PIDController(0.25, 0, 0.1);
    public static final Force tipDeadband = Newtons.of(3);
  }

  public static class VisionConstants {
    public static final Distance kFieldWidth = Meters.of(16.54);
    public static final Distance kFieldHeight = Meters.of(8.229);

    public static final Distance minCamDistToTag = Meters.of(0.5);
    public static final Distance maxCamDistToTag = Meters.of(0.8);
    public static final Distance maxVertDisp = Meters.of(0.2);

    public static final double camChassisXOffset = 0.0;
    public static final double camChassisYOffset = -0.254;
    public static final double camChassisZOffset = 0.1143;

    public static final Angle minAngError = Degrees.of(5);
    public static final Distance minTransError = Meters.of(0.05);

    // TODO placeholder offsets, need tuning
    public static final Distance tagXOffset = Meters.of(0.1);
    public static final Distance tagYOffset = Meters.of(0.2);

    public static final double tagDistSetpoint = 0.1;
    public static final double kCameraPitchRadians = Math.PI / 3;
    public static final Transform3d robotToCamOne =
        new Transform3d(
            new Translation3d(camChassisXOffset, camChassisYOffset, camChassisZOffset),
            new Rotation3d(0, kCameraPitchRadians, 0));
    public static final Transform3d robotToCamTwo =
        new Transform3d(
            new Translation3d(camChassisXOffset, -camChassisYOffset, camChassisZOffset),
            new Rotation3d(0, kCameraPitchRadians, 0));

    public static final Matrix<N3, N1> kMultiTagStdDevs =
        VecBuilder.fill(0.5, 0.5, Double.POSITIVE_INFINITY);

    public static AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  }

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
}
