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

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Newtons;

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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import java.util.List;
import java.util.Map;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

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
    public static final double DRIVE_DEADBAND = 0.02;
    public static final Distance TAG_DISTANCE = Meters.of(0.3);
    public static final PIDController translationController = new PIDController(0.2, 0, 0.15);
    public static final PIDController rotationController = new PIDController(0.3, 0, 0);

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

    public static final double camChassisXOffset = 0.3;
    public static final double camChassisYOffset = 0.3;
    public static final double camChassisZOffset = 0.0;

    public static final double tagDistSetpoint = 0.0;
    public static final double kCameraHeight = 0.04;
    public static final double kCameraPitchRadians = 0;
    public static final Transform3d robotToCam =
        new Transform3d(
            new Translation3d(camChassisXOffset, camChassisYOffset, camChassisZOffset),
            new Rotation3d(0, 0, 0));

    public static final Matrix<N3, N1> kMultiTagStdDevs =
        VecBuilder.fill(0.5, 0.5, Double.POSITIVE_INFINITY);

    public static final AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    public static final PhotonCamera aprilCamOne = new PhotonCamera("aprilOne");
    public static final PhotonCamera aprilCamTwo = new PhotonCamera("aprilTwo");

    public static final PhotonPoseEstimator poseEstimatorOne =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.robotToCam);
    public static final PhotonPoseEstimator poseEstimatorTwo =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.robotToCam); // TODO need diff config for second camera

    // a map of tag ids to corresponding tag heights on the field
    public static final Map<List<Integer>, Distance> tagToHeight =
        Map.of(
            List.of(1, 2, 12, 13), Meters.of(1.40335), // coral station
            List.of(3, 16), Meters.of(1.216152), // processor
            List.of(4, 5, 14, 15), Meters.of(1.796542), // barge
            List.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22), Meters.of(0.22225) // reef
            );
  }
}
