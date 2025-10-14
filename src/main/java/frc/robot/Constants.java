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
import static frc.robot.util.Elastic.Notification.NotificationLevel.ERROR;
import static frc.robot.util.Elastic.Notification.NotificationLevel.WARNING;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.SnapToTarget.TargetPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.PhotonPoseEstimatorNameTuple;
import frc.robot.util.faultChecker.Fault;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import java.util.stream.IntStream;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.Logger;
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
  public static final Alliance defaultAlliance = Alliance.Blue;

  // Physical values of the robot
  public static final Mass ROBOT_MASS_KG = Pounds.of(138.308);
  // Converted form lbs in2 because wpi doesnt have a unit
  public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(5.674);
  public static final double WHEEL_COF = 1.2;
  public static final double kDt = currentMode == Mode.SIM ? 0.005 : 0.02;

  public static final Pose2d startPose = new Pose2d(new Translation2d(5, 1), new Rotation2d());

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum GameState {
    L1_SCORE,
    L2_SCORE,
    L3_SCORE,
    L4_SCORE,
    L2_ALGAE,
    L3_ALGAE,
    HUMAN_PLAYER_STATION,
    NONE,
  }

  public static record AngleAndDistance(Angle angle, Distance distance) {}

  public static Map<GameState, AngleAndDistance> reefMap = getReefMap();

  private static Map<GameState, AngleAndDistance> getReefMap() {
    switch (currentMode) {
      default:
        return Map.of(
            GameState.L1_SCORE, new AngleAndDistance(Rotations.of(-0.2), Inches.of(7.3)),
            GameState.L2_SCORE, new AngleAndDistance(Rotations.of(0.2), Inches.of(13.3)),
            GameState.L3_SCORE, new AngleAndDistance(Rotations.of(0.2), Inches.of(28)),
            GameState.L4_SCORE, new AngleAndDistance(Rotations.of(0.15), Inches.of(60.9)),
            GameState.L2_ALGAE, new AngleAndDistance(Degrees.of(32), Inches.of(14)),
            GameState.L3_ALGAE, new AngleAndDistance(Degrees.of(32), Inches.of(35)),
            GameState.HUMAN_PLAYER_STATION,
                new AngleAndDistance(Rotations.of(-0.15), Inches.of(17)),
            GameState.NONE, new AngleAndDistance(Rotations.of(-0.25), Inches.of(0)));
      case SIM:
        return Map.of(
            GameState.L1_SCORE, new AngleAndDistance(Rotations.of(-0.2), Inches.of(7.3)),
            GameState.L2_SCORE, new AngleAndDistance(Rotations.of(0.2), Inches.of(14.8)),
            GameState.L3_SCORE, new AngleAndDistance(Rotations.of(0.2), Inches.of(29.5)),
            GameState.L4_SCORE, new AngleAndDistance(Rotations.of(0.1), Inches.of(62)),
            GameState.L2_ALGAE, new AngleAndDistance(Degrees.of(32), Inches.of(7)),
            GameState.L3_ALGAE, new AngleAndDistance(Degrees.of(32), Inches.of(15)),
            GameState.HUMAN_PLAYER_STATION,
                new AngleAndDistance(Rotations.of(-0.09), Inches.of(10)),
            GameState.NONE, new AngleAndDistance(Rotations.of(-0.25), Inches.of(0)));
    }
  }

  public static Distance[] elevatorStateMap = {Inches.of(0), Inches.of(0), Inches.of(0)};

  public static class DriveConstants {
    public static final Distance kChassisSize = Feet.of(2.853);
    public static final LinearVelocity kMaxPathSpeed = TunerConstants.kSpeedAt12Volts;
    public static final LinearAcceleration kMaxTranslationAcceleration =
        MetersPerSecondPerSecond.of(6);
    public static final AngularVelocity kMaxRotVelocity = RadiansPerSecond.of(4 * Math.PI);
    public static final AngularAcceleration kMaxRotAcceleration =
        RadiansPerSecondPerSecond.of(3 * Math.PI);
    public static final double kDriveDeadband = 0.02;
    public static final Distance kTagDistance = Meters.of(0.3);
    public static final PIDController kTranslationController = new PIDController(0.5, 0.001, 0);
    public static final PIDController kRotationController = new PIDController(0.03, 0.01, 0);

    public static final DriveTrainSimulationConfig mapleSimConfig =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(ROBOT_MASS_KG)
            .withCustomModuleTranslations(Drive.getModuleTranslations())
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(
                new SwerveModuleSimulationConfig(
                    DCMotor.getKrakenX60(1),
                    DCMotor.getFalcon500(1),
                    TunerConstants.FrontLeft.DriveMotorGearRatio,
                    TunerConstants.FrontLeft.SteerMotorGearRatio,
                    Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                    Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                    Meters.of(TunerConstants.FrontLeft.WheelRadius),
                    KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                    WHEEL_COF));
  }

  public static class ManipulatorConstants {
    public static final int kPivotId = 5;
    public static final int kRollerId = 6;
    public static final int kBeamBreakPort = 9;

    public static final Angle kRotTolerance = Rotations.of(0.05);
    public static final double kDefaultRollerSpeed = 0.35;
    public static final double kHumanPlayerStationSpeed = 0.5;
    public static final double kWhilePivotingSpeed = 0.0;
    public static final double kP = 2;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kSimP = 8;
    public static final double kSimI = 0.025;
    public static final double kSimD = 0.3;

    public static final int currentLimit = 40;
    public static final Pose3d kStartingPose =
        new Pose3d(0.2539295, 0, 0.4757015, new Rotation3d());
    public static final Pose3d kCoralPose = new Pose3d(0.26, 0, 0.395691, new Rotation3d());
  }

  public static class VisionConstants {
    public static final Distance kFieldWidth = Meters.of(16.54);
    public static final Distance kFieldHeight = Meters.of(8.229);

    public static final Distance kMinCamDistToTag = Meters.of(0);
    public static final Distance kMaxCamDistToTag = Meters.of(4);
    public static final Distance kMaxVertDisp = Meters.of(3);
    public static final double kMaxTagAmbiguity = 0.2;
    public static final Angle kRollBounds = Radians.of(0.2);
    public static final Angle kPitchBounds = Radians.of(0.2);

    private static final Distance halfOffset = DriveConstants.kChassisSize.div(2);
    public static final Distance kCamOneChassisXOffset = Meters.zero();
    public static final Distance kCamOneChassisYOffset = Feet.of(2);
    public static final Distance kCamTwoChassisXOffset = Meters.zero();
    public static final Distance kCamTwoChassisYOffset = Feet.of(2);
    public static final Distance KCamChassisZOffset = Feet.of(1.8);

    public static final Angle kMinAngError = Degrees.of(5);
    public static final Distance kMinTransError = Meters.of(0.05);

    public static final Distance kTagXOffset = Meters.of(0.15);
    public static final Distance kTagYOffset = halfOffset;

    public static final Distance tagDistSetpoint = Meters.of(0.1);
    public static final Angle kCameraPitch = Degrees.of(0);
    public static final Transform3d kRobotToCamOne =
        new Transform3d(
            new Translation3d(kCamOneChassisXOffset, kCamOneChassisYOffset, KCamChassisZOffset),
            new Rotation3d(
                0,
                kCameraPitch.in(Radians),
                new Translation2d(kCamOneChassisXOffset, kCamOneChassisYOffset)
                    .getAngle()
                    .getRadians()));
    public static final Transform3d kRobotToCamTwo =
        new Transform3d(
            new Translation3d(kCamTwoChassisXOffset, kCamTwoChassisYOffset, KCamChassisZOffset),
            new Rotation3d(
                0,
                kCameraPitch.in(Radians),
                new Translation2d(kCamTwoChassisXOffset, kCamTwoChassisYOffset)
                    .getAngle()
                    .getRadians()));

    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.25, 0.25, 0.5);

    public static final AprilTagFieldLayout kAprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static final PhotonPoseEstimator poseEstimatorOne =
        new PhotonPoseEstimator(
            VisionConstants.kAprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.kRobotToCamOne);
    public static final PhotonPoseEstimator poseEstimatorTwo =
        new PhotonPoseEstimator(
            VisionConstants.kAprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.kRobotToCamTwo);
    public static final List<PhotonPoseEstimatorNameTuple> estimAndCam =
        List.of(
            new PhotonPoseEstimatorNameTuple("aprilOne", poseEstimatorOne),
            new PhotonPoseEstimatorNameTuple("aprilTwo", poseEstimatorTwo));

    public static final List<TargetPose> kTargetPoses = generateTargetPoses();

    private static List<TargetPose> generateTargetPoses() {
      List<TargetPose> poses = new ArrayList<>();
      poses.addAll(generateTargetPosesForOneAlliance(Alliance.Blue));
      poses.addAll(generateTargetPosesForOneAlliance(Alliance.Red));
      return poses;
    }

    private static List<TargetPose> generateTargetPosesForOneAlliance(Alliance alliance) {
      int startTagId = alliance == Alliance.Blue ? 17 : 6;
      List<Pose2d> apriltagPoses =
          IntStream.rangeClosed(startTagId, startTagId + 5)
              .mapToObj(id -> kAprilTagFieldLayout.getTagPose(id).orElse(null))
              .filter(pose3d -> pose3d != null)
              .map(Pose3d::toPose2d)
              .toList();

      List<TargetPose> targetPoses = new ArrayList<>();
      for (Pose2d pose : apriltagPoses) {
        targetPoses.add(
            new TargetPose(
                convertApriltagPoseToTargetPose(pose, false, kTagXOffset, kTagYOffset),
                alliance,
                true,
                false));
        targetPoses.add(
            new TargetPose(
                convertApriltagPoseToTargetPose(pose, true, kTagXOffset, kTagYOffset),
                alliance,
                true,
                true));
      }
      startTagId = alliance == Alliance.Blue ? 12 : 1;
      apriltagPoses =
          IntStream.rangeClosed(startTagId, startTagId + 1)
              .mapToObj(id -> kAprilTagFieldLayout.getTagPose(id).orElse(null))
              .filter(pose3d -> pose3d != null)
              .map(Pose3d::toPose2d)
              .toList();
      for (Pose2d pose : apriltagPoses) {
        targetPoses.add(
            new TargetPose(
                convertApriltagPoseToTargetPose(pose, false, Meters.zero(), kTagYOffset),
                alliance,
                false,
                false));
      }
      return targetPoses;
    }

    private static Pose2d convertApriltagPoseToTargetPose(
        Pose2d pose, boolean right, Distance xOffset, Distance yOffset) {
      var rotatedPose =
          new Pose2d(
                  new Translation2d(yOffset, right ? xOffset : xOffset.unaryMinus()),
                  new Rotation2d())
              .rotateBy(pose.getRotation());
      return new Pose2d(
          new Translation2d(
              rotatedPose.getMeasureX().plus(pose.getMeasureX()),
              rotatedPose.getMeasureY().plus(pose.getMeasureY())),
          pose.getRotation().rotateBy(Rotation2d.k180deg));
    }
  }

  public static class ClimberConstants {
    public static final int kClimberMotorID = 2;
    public static final double kClimberMotorMult = 0.9;

    public static final double kP = 1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static enum State {
      TOP,
      BOTTOM,
      MID,
    }

    public static final Map<State, Angle> stateMap =
        Map.of(
            State.TOP,
            Rotations.of(0),
            State.MID,
            Rotations.of(0.255),
            State.BOTTOM,
            Rotations.of(0.285));
    public static final Angle setpointTolerance = Rotations.of(0.05);

    public static final Pose3d kStartingPose = new Pose3d(-0.278, 0, 0.355, new Rotation3d());
  }

  public static class ElevatorConstants {
    public static final Distance kStage1RideDist = Millimeters.of(660.4);

    public static final Distance kStage1MaxHeight = kStage1RideDist;
    public static final Distance kStage2RideDist = Millimeters.of(657.225);
    public static final Distance kStage2MaxHeight = kStage1MaxHeight.plus(kStage2RideDist);
    public static final Distance kCarriageRideDist = Millimeters.of(400.05);
    public static final Distance kCarriageMaxHeight = kStage2MaxHeight.plus(kCarriageRideDist);

    public static final String canBus = "blinky";
    public static final int kTalonLeaderCANId = 14;
    public static final int kTalonFollowerCANId = 15;
    public static final Distance kMaxHeight = kCarriageMaxHeight;
    public static final Distance kSetpointTolerance = Inches.of(0.5);
    public static final AngularVelocity kMaxVelocity = RotationsPerSecond.of(100);
    public static final AngularAcceleration kMaxAcceleration = RotationsPerSecondPerSecond.of(10);
    public static final Velocity<AngularAccelerationUnit> kMaxJerk =
        RotationsPerSecondPerSecond.per(Second).of(25);
    public static final Angle rotVelTolerance = Rotations.of(0.05);

    public static final Mass kElevatorMass = Pounds.of(26.9145454);

    // 3 to 1 ratio on the motor
    public static final int kGearRatio = 3;
    public static final Distance kPullyRadius = Millimeters.of(28.5);
    public static final Angle kFullExtensionAngle =
        Radians.of(kMaxHeight.div(kPullyRadius).magnitude()).times(3);

    // Ratio of height to angle
    public static final Per<DistanceUnit, AngleUnit> kAngularSpan =
        kMaxHeight.div(kFullExtensionAngle);

    // Ratio of angle to height
    public static final Per<AngleUnit, DistanceUnit> kSpanAngle =
        kFullExtensionAngle.div(kMaxHeight);

    public static final double kS = 0.097565;
    public static final double kG0 = 0.68798;
    public static final double kV = 0.1293;
    public static final double kA = 0.025;
    public static final double kP = 0.16037;
    public static final double kI = 0.1;
    public static final double kD = 0;

    public static final double kSimS = 0;
    public static final double kSimG0 = 0.729;
    public static final double kSimV = 0.126;
    public static final double kSimA = 0.005;

    public static final double kSimP = 0.5;
    public static final double kSimI = 0;
    public static final double kSimD = 0;

    public static final FeedbackSensor feedbackSensor = FeedbackSensor.kAbsoluteEncoder;
    public static final double kFF = 0;
    public static final int currentLimit = 70;

    // public static final double kS = 0;
    // public static final double kG0 = 40;
    // public static final double kV = 8;

    // public static final double kG1 = 40;
    // public static final double kG2 = 40;

    public static final Slot0Configs elevatorGains0 =
        new Slot0Configs()
            .withKP(kP)
            .withKI(kI)
            .withKD(kD)
            .withKS(kS)
            .withKV(kV)
            .withKA(kA)
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(kG0);

    public static final Slot0Configs elevatorSimGains0 =
        new Slot0Configs()
            .withKP(kSimP)
            .withKI(kSimI)
            .withKD(kSimD)
            .withKS(kSimS)
            .withKV(kSimV)
            .withKA(kSimA)
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(kSimG0);

    public static final TalonFXConfiguration elevatorConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Amps.of(currentLimit))
                    .withSupplyCurrentLimitEnable(true));

    public static final SysIdRoutine.Config sysIdConfig =
        new SysIdRoutine.Config(
            Volts.per(Second).of(1),
            Volts.of(6),
            null,
            (state) -> Logger.recordOutput("Elevator/sysIdState", state.toString()));
    public static final Pose3d kStage1StartPos =
        new Pose3d(0.1017225, 0, 0.094701406, new Rotation3d());
    public static final Pose3d kStage2StartPos =
        new Pose3d(0.1017225, 0, 0.120101, new Rotation3d());
    public static final Pose3d kCarriageStartPos =
        new Pose3d(0.1017225, 0, 0.145501, new Rotation3d());
  }

  public static class FaultConstants {
    public static List<Fault> getTalonFXFaults(TalonFX talon) {
      return new ArrayList<Fault>() {
        {
          add(
              new Fault(
                  () -> (talon.getFault_BootDuringEnable().getValue()),
                  ERROR,
                  "Robot enabled during motor bootup"));
          add(
              new Fault(
                  () -> (talon.getFault_BridgeBrownout().getValue()),
                  ERROR,
                  "Motor bridge brownout"));
          add(new Fault(() -> (talon.getFault_DeviceTemp().getValue()), ERROR, "Motor over temp"));
          add(
              new Fault(
                  () -> (talon.getFault_ForwardHardLimit().getValue()),
                  WARNING,
                  "Motor reached forward hard limit"));
          add(
              new Fault(
                  () -> (talon.getFault_ForwardSoftLimit().getValue()),
                  WARNING,
                  "Motor reached forward soft limit"));
          add(
              new Fault(
                  () -> (talon.getFault_FusedSensorOutOfSync().getValue()),
                  WARNING,
                  "Fused sensor resync. May cause a jump in location"));
          add(new Fault(() -> (talon.getFault_Hardware().getValue()), ERROR, "Hardware fault"));
          add(
              new Fault(
                  () -> (talon.getFault_MissingDifferentialFX().getValue()),
                  ERROR,
                  "Canont reach remote differential talon"));
          add(
              new Fault(
                  () -> (talon.getFault_MissingHardLimitRemote().getValue()),
                  ERROR,
                  "Canont reach remote hard stop limit switch"));
          add(
              new Fault(
                  () -> (talon.getFault_MissingSoftLimitRemote().getValue()),
                  ERROR,
                  "Cannot reach remote soft limit device"));
          add(
              new Fault(
                  () -> (talon.getFault_OverSupplyV().getValue()),
                  ERROR,
                  "Supply voltage to high"));
          add(
              new Fault(
                  () -> (talon.getFault_ProcTemp().getValue()), ERROR, "Processor over temp"));
          add(
              new Fault(
                  () -> (talon.getFault_RemoteSensorDataInvalid().getValue()),
                  ERROR,
                  "Remote sensor data no longer trusted. may be caused by a drop off the can bus or other fault in the sensor"));
          add(
              new Fault(
                  () -> (talon.getFault_RemoteSensorPosOverflow().getValue()),
                  ERROR,
                  "Remote sensor position has exceeded the size of a status signal frame"));
          add(
              new Fault(
                  () -> (talon.getFault_RemoteSensorReset().getValue()),
                  ERROR,
                  "Remote sensor has been reset"));
          add(
              new Fault(
                  () -> (talon.getFault_ReverseHardLimit().getValue()),
                  WARNING,
                  "Motor has reached reverse hard limit"));
          add(
              new Fault(
                  () -> (talon.getFault_ReverseSoftLimit().getValue()),
                  WARNING,
                  "Motor has reached reverse soft limit"));
          add(
              new Fault(
                  () -> (talon.getFault_StaticBrakeDisabled().getValue()),
                  ERROR,
                  "Static break overloaded. Will be temporarily disabled"));
          add(
              new Fault(
                  () -> (talon.getFault_StatorCurrLimit().getValue()),
                  ERROR,
                  "Stator current limit reached"));
          add(
              new Fault(
                  () -> (talon.getFault_Undervoltage().getValue()), ERROR, "Motor undervoltage"));
          add(
              new Fault(
                  () -> (talon.getFault_UnlicensedFeatureInUse().getValue()),
                  ERROR,
                  "Unlicenced feature in use"));
          add(
              new Fault(
                  () -> (talon.getFault_UnstableSupplyV().getValue()),
                  ERROR,
                  "Supply voltage unstable"));
          add(
              new Fault(
                  () -> (talon.getFault_UsingFusedCANcoderWhileUnlicensed().getValue()),
                  ERROR,
                  "Using a fused CANcoder without pheonix pro"));
        }
      };
    }

    public static List<Fault> getPigeonFaults(Pigeon2 pigeon) {
      return new ArrayList<Fault>() {
        {
          add(
              new Fault(
                  () -> (pigeon.getFault_BootDuringEnable().getValue()),
                  ERROR,
                  "Robot enabled during pigeon boot"));
          add(
              new Fault(
                  () -> (pigeon.getFault_BootIntoMotion().getValue()),
                  ERROR,
                  "Motion during pigeon boot"));
          add(
              new Fault(
                  () -> (pigeon.getFault_BootupAccelerometer().getValue()),
                  ERROR,
                  "Accelerometer bootup failed"));
          add(
              new Fault(
                  () -> (pigeon.getFault_BootupGyroscope().getValue()),
                  ERROR,
                  "Gyro bootup failed"));
          add(
              new Fault(
                  () -> (pigeon.getFault_BootupMagnetometer().getValue()),
                  ERROR,
                  "Magnetometer bootup failed"));
          add(
              new Fault(
                  () -> (pigeon.getFault_DataAcquiredLate().getValue()),
                  WARNING,
                  "Motion data acquired late"));
          add(new Fault(() -> (pigeon.getFault_Hardware().getValue()), ERROR, "Hardware fault"));
          add(
              new Fault(
                  () -> (pigeon.getFault_LoopTimeSlow().getValue()),
                  WARNING,
                  "Motion data collection slow"));
          add(
              new Fault(
                  () -> (pigeon.getFault_SaturatedAccelerometer().getValue()),
                  ERROR,
                  "Accelerometer saturated"));
          add(
              new Fault(
                  () -> (pigeon.getFault_SaturatedGyroscope().getValue()),
                  ERROR,
                  "Gyro staturated"));
          add(
              new Fault(
                  () -> (pigeon.getFault_SaturatedMagnetometer().getValue()),
                  ERROR,
                  "Magnetometer staturated"));
          add(
              new Fault(
                  () -> (pigeon.getFault_Undervoltage().getValue()), ERROR, "Voltage to low"));
          add(
              new Fault(
                  () -> (pigeon.getFault_UnlicensedFeatureInUse().getValue()),
                  ERROR,
                  "Unlicenced feature in use"));
        }
      };
    }

    public static List<Fault> getCANCoderFaults(CANcoder CANcoder) {
      return new ArrayList<Fault>() {
        {
          add(new Fault(() -> (CANcoder.getFault_BadMagnet().getValue()), ERROR, "Bad magnet"));
          add(
              new Fault(
                  () -> (CANcoder.getFault_BootDuringEnable().getValue()),
                  ERROR,
                  "Robot enabled during pigeon boot"));
          add(new Fault(() -> (CANcoder.getFault_Hardware().getValue()), ERROR, "Hardware fault"));
          add(
              new Fault(
                  () -> (CANcoder.getFault_Undervoltage().getValue()),
                  ERROR,
                  "Supply voltage to low"));
          add(
              new Fault(
                  () -> (CANcoder.getFault_UnlicensedFeatureInUse().getValue()),
                  ERROR,
                  "Unlicenced feature in use"));
        }
      };
    }

    public static List<Fault> getSparkFaults(Supplier<Faults> faults, Supplier<Warnings> warnings) {
      return new ArrayList<Fault>() {
        {
          add(new Fault(() -> (faults.get().can), ERROR, "Can fault"));
          add(new Fault(() -> (faults.get().escEeprom), ERROR, "escEeprom fault"));
          add(new Fault(() -> (faults.get().firmware), ERROR, "Firmware fault"));
          add(new Fault(() -> (faults.get().gateDriver), ERROR, "Gate driver fault"));
          add(new Fault(() -> (faults.get().motorType), ERROR, "Incorrect motor type"));
          add(new Fault(() -> (faults.get().other), ERROR, "Other Fault"));
          add(new Fault(() -> (faults.get().sensor), ERROR, "Sensor fault"));
          add(new Fault(() -> (faults.get().temperature), ERROR, "Over temp"));

          add(new Fault(() -> (warnings.get().brownout), ERROR, "Brownout detected"));
          add(new Fault(() -> (warnings.get().escEeprom), ERROR, "escEeprom warning"));
          add(new Fault(() -> (warnings.get().extEeprom), ERROR, "extEeprom warning"));
          add(new Fault(() -> (warnings.get().other), ERROR, "Other warning"));
          add(new Fault(() -> (warnings.get().overcurrent), ERROR, "Over current"));
          add(new Fault(() -> (warnings.get().sensor), ERROR, "Sensor warning"));
          add(new Fault(() -> (warnings.get().stall), WARNING, "Stall detected"));
          add(new Fault(() -> (warnings.get().hasReset), WARNING, "SparkMax reset"));
        }
      };
    }
  }

  public static class PDHConstants {
    public static final Current kMaxTotalCurrentDraw = Amps.of(120);
    public static final Current kMaxHighChannelDraw = Amps.of(40);
    public static final Current kMaxLowChannelDraw = Amps.of(15);
    public static final Temperature kMaxTemperature = Celsius.of(85);
    public static final Voltage kMaxInputVoltage = Volts.of(16);
    public static final Voltage kBatteryVoltage = Volts.of(12);
    public static final double kBatteryCapacityAmpHours =
        18; // I wasnt able to find a way to have a unit of electric charge
    public static final Energy kBatteryEnergy =
        Joules.of(
            (kBatteryVoltage.in(Volts) * kBatteryCapacityAmpHours)
                * 60
                * 60); // * 60 * 60 is to convert from watt hours to Joles

    public static List<Fault> getPDHFaults(
        Supplier<PowerDistributionFaults> faultSupplier, PowerDistribution robotPower) {
      List<Fault> faults =
          new ArrayList<Fault>() {
            {
              add(new Fault(() -> (faultSupplier.get().Brownout), ERROR, "Brownout"));
              add(new Fault(() -> (faultSupplier.get().CanWarning), ERROR, "Can warning"));
              add(new Fault(() -> (faultSupplier.get().HardwareFault), ERROR, "Hardware fault"));

              add(
                  new Fault(
                      () ->
                          (Amps.of(robotPower.getTotalCurrent())
                              .gte(PDHConstants.kMaxTotalCurrentDraw)),
                      ERROR,
                      "Over current detected"));
              add(
                  new Fault(
                      () -> (Volts.of(robotPower.getVoltage()).gte(PDHConstants.kMaxInputVoltage)),
                      ERROR,
                      "Over voltage detected"));
              add(
                  new Fault(
                      () ->
                          (Celsius.of(robotPower.getTemperature())
                              .gte(PDHConstants.kMaxTemperature)),
                      ERROR,
                      "Over tempreture detected"));
            }
          };

      for (int i = 0; i < 23; i++) {
        final int finalI = i;
        faults.add(
            new Fault(
                () -> (faultSupplier.get().getBreakerFault(finalI)),
                ERROR,
                "Breaker fault on channel" + Integer.toString(i)));
      }

      return faults;
    }
  }
}
