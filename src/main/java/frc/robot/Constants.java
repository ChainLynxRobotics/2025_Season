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

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode currentMode =
      Mode.REAL; // RobotBase.isReal() ? Mode.REAL : Mode.SIM; robot thinks it's fake wut

  // Phisical values of the robot
  public static final Mass ROBOT_MASS_KG = Kilogram.of(74.088);
  public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(6.883);
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
    // TODO verify actual chassis size
    public static final Distance chassisSize = Meters.of(0.9);
    public static final LinearVelocity maxTranslationSpeed = MetersPerSecond.of(10);
    public static final AngularVelocity maxRotVelocity = RadiansPerSecond.of(4 * Math.PI);
    public static final double DRIVE_DEADBAND = 0.05;
    public static final Distance TAG_DISTANCE = Meters.of(0.3);
    public static final PIDController translationController = new PIDController(0.5, 0.001, 0);
    public static final PIDController rotationController = new PIDController(0.03, 0.01, 0);

    public static final PIDController tipController = new PIDController(0.25, 0, 0.1);
    public static final double tipFF = 0.1;
    public static final Force tipDeadband = Newtons.of(3);
  }

  public static class VisionConstants {
    public static final Distance kFieldWidth = Meters.of(16.54);
    public static final Distance kFieldHeight = Meters.of(8.229);

    public static final Distance kMinCamDistToTag = Meters.of(0.1);
    public static final Distance kMaxCamDistToTag = Meters.of(2.0);
    public static final Distance kMaxVertDisp = Meters.of(2.5);
    public static final Angle kRollBounds = Radians.of(0.2);
    public static final Angle kPitchBounds = Radians.of(0.2);

    public static final Distance kCamChassisXOffset = Inches.of(0.0);
    public static final Distance kCamChassisYOffset = Inches.of(-10);
    public static final Distance KCamChassisZOffset = Inches.of(4.5);

    public static final Angle kMinAngError = Degrees.of(5);
    public static final Distance kMinTransError = Meters.of(0.05);

    // TODO placeholder offsets, need tuning
    public static final Distance kTagXOffset = Meters.of(0.1);
    public static final Distance kTagYOffset = Meters.of(0.2);

    public static final Distance tagDistSetpoint = Meters.of(0.1);
    public static final Angle kCameraPitch = Radians.of(Math.PI / 3);
    public static final Transform3d robotToCamOne =
        new Transform3d(
            new Translation3d(kCamChassisXOffset, kCamChassisYOffset, KCamChassisZOffset),
            new Rotation3d(0, kCameraPitch.in(Radians), 0));
    public static final Transform3d robotToCamTwo =
        new Transform3d(
            new Translation3d(
                kCamChassisXOffset, kCamChassisYOffset.unaryMinus(), KCamChassisZOffset),
            new Rotation3d(0, kCameraPitch.in(Radians), 0));

    public static final Matrix<N3, N1> kMultiTagStdDevs =
        VecBuilder.fill(0.5, 0.5, Double.POSITIVE_INFINITY);

    public static AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  }
}
