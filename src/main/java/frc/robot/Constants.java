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
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Energy;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.faultChecker.Fault;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

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
  public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(0.0588);
  public static final double WHEEL_COF = 1.75;
  public static final double kDt = 0.02;

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

  public static enum ElevatorState {
    STAGE1,
    STAGE2,
    STAGE3;
  }

  public static record AngleAndDistance(Angle angle, Distance distance) {}

  public static Map<GameState, AngleAndDistance> reefMap =
      Map.of(
          GameState.L1_SCORE, new AngleAndDistance(Rotations.of(0.45), Inches.of(7.3)),
          GameState.L2_SCORE, new AngleAndDistance(Rotations.of(0.05), Inches.of(21.3)),
          GameState.L3_SCORE, new AngleAndDistance(Rotations.of(0.05), Inches.of(35)),
          GameState.L4_SCORE, new AngleAndDistance(Rotations.of(0.1), Inches.of(64.9)),
          GameState.L2_ALGAE, new AngleAndDistance(Degrees.of(58), Inches.of(7)),
          GameState.L3_ALGAE, new AngleAndDistance(Degrees.of(58), Inches.of(15)),
          GameState.HUMAN_PLAYER_STATION, new AngleAndDistance(Rotations.of(0.34), Inches.of(10)),
          GameState.NONE, new AngleAndDistance(Rotations.of(0.4), Inches.of(0)));

  public static Distance[] elevatorStateMap = {Inches.of(0), Inches.of(0), Inches.of(0)};

  public static class DriveConstants {
    public static final Distance kChassisSize = Inches.of(34.24);
    public static final LinearVelocity kMaxPathSpeed = MetersPerSecond.of(10);
    public static final LinearAcceleration kMaxTranslationAcceleration =
        MetersPerSecondPerSecond.of(6);
    public static final AngularVelocity kMaxRotVelocity = RadiansPerSecond.of(4 * Math.PI);
    public static final AngularAcceleration kMaxRotAcceleration =
        RadiansPerSecondPerSecond.of(3 * Math.PI);
    public static final double kDriveDeadband = 0.02;
    public static final Distance kTagDistance = Meters.of(0.3);
    public static final PIDController kTranslationController = new PIDController(0.5, 0.001, 0);
    public static final PIDController kRotationController = new PIDController(0.03, 0.01, 0);

    public static final PIDController tipController = new PIDController(0.25, 0, 0.1);
    public static final double tipFF = 0.1;
    public static final Force tipDeadband = Newtons.of(3);
  }

  public static class ManipulatorConstants {
    public static final int kPivotId = 5;
    public static final int kRollerId = 6;
    public static final int kBeamBreakPort = 9;

    public static final Angle kRotTolerance = Rotations.of(0.05);
    public static final double kDefaultRollerSpeed = 0.2;
    public static final double kHumanPlayerStationSpeed = 0.5;
    public static final double kWhilePivotingSpeed = 0.05;
    public static final double kManualPivotSpeed = 0.3;
    public static final Time kDefaultPickupTime = Seconds.of(1);
    public static final double kP = 2;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 0;
    public static final int currentLimit = 40;
  }

  public static class VisionConstants {
    public static final Distance kFieldWidth = Meters.of(16.54);
    public static final Distance kFieldHeight = Meters.of(8.229);

    public static final Distance kMinCamDistToTag = Meters.of(0.1);
    public static final Distance kMaxCamDistToTag = Meters.of(2.0);
    public static final Distance kMaxVertDisp = Meters.of(2.5);
    public static final double kMaxTagAmbiguity = 0.25;
    public static final Angle kRollBounds = Radians.of(0.2);
    public static final Angle kPitchBounds = Radians.of(0.2);

    private static final double halfOffset = DriveConstants.kChassisSize.in(Units.Inches) / 2;
    public static final Distance kCamOneChassisXOffset = Inches.of(halfOffset);
    public static final Distance kCamOneChassisYOffset = Inches.of(halfOffset);
    public static final Distance kCamTwoChassisXOffset = Inches.of(-halfOffset);
    public static final Distance kCamTwoChassisYOffset = Inches.of(-halfOffset);
    public static final Distance KCamChassisZOffset = Inches.of(4.5);

    public static final Angle kMinAngError = Degrees.of(5);
    public static final Distance kMinTransError = Meters.of(0.05);

    // TODO placeholder offsets, need tuning
    public static final Distance kTagXOffset = Meters.of(0.1);
    public static final Distance kTagYOffset = Meters.of(0.2);

    public static final Distance tagDistSetpoint = Meters.of(0.1);
    public static final Angle kCameraPitch = Radians.of(Math.PI / 3);
    public static final Transform3d kRobotToCamOne =
        new Transform3d(
            new Translation3d(kCamOneChassisXOffset, kCamOneChassisYOffset, KCamChassisZOffset),
            new Rotation3d(0, kCameraPitch.in(Radians), 0));
    public static final Transform3d kRobotToCamTwo =
        new Transform3d(
            new Translation3d(
                kCamTwoChassisXOffset, kCamTwoChassisYOffset.unaryMinus(), KCamChassisZOffset),
            new Rotation3d(0, kCameraPitch.in(Radians), 0));

    public static final Matrix<N3, N1> kMultiTagStdDevs =
        VecBuilder.fill(0.5, 0.5, Double.POSITIVE_INFINITY);

    public static final AprilTagFieldLayout kAprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  }

  public static class ClimberConstants {
    public static final int kClimberMotorID = 2;
    public static final int kUpperLimSwitchId = 0;
    public static final int kLowerLimSwitchId = 1;
    public static final int kMidBeamBreakId = 2;
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
  }

  public static class ElevatorConstants {
    public static final String canBus = "blinky";
    public static final int kTalonLeaderCANId = 14;
    public static final int kTalonFollowerCANId = 15;
    public static final Distance kMaxHeight = Meters.of(1.72);
    public static final double kDeadReckoningSpeed = 0.1;
    public static final double kDeadRecogningDeadZone = 0.05;
    public static final double kRestInput = 0.02;
    public static final Angle kSetpointTolerance = Rotations.of(0.5);
    public static final AngularVelocity kMaxVelocity = RotationsPerSecond.of(15);
    public static final AngularAcceleration kMaxAcceleration = RotationsPerSecondPerSecond.of(10);
    public static final Angle rotVelTolerance = Rotations.of(0.05);

    public static double totalExtensionTime = kMaxHeight.in(Units.Meters) / kDeadReckoningSpeed;

    public static final Distance kEncoderOffset = Meters.of(0);

    public static final boolean kIsInverted = true;

    // 3 to 1 ratio on the motor
    public static final int kGearRatio = 3;
    public static final Angle kFullExtensionAngle = Rotations.of(9.56 * kGearRatio);

    // Ratio of height to angle
    public static final Per<DistanceUnit, AngleUnit> kAngularSpan =
        kMaxHeight.div(kFullExtensionAngle);

    // Ratio of angle to height
    public static final Per<AngleUnit, DistanceUnit> kSpanAngle =
        kFullExtensionAngle.div(kMaxHeight);

    public static final double kPositionConversionFactor = 1 / kFullExtensionAngle.in(Rotations);
    public static final double kS = 0.097565;
    public static final double kG0 = 0.68798;
    public static final double kG1 = 0.68798;
    public static final double kG2 = 0.68798;
    public static final double kV = 0.1293;
    public static final double kA = 0.025;

    public static final FeedbackSensor feedbackSensor = FeedbackSensor.kAbsoluteEncoder;
    public static final double kP = 0.16037;
    public static final double kI = 0.1;

    public static final double kD = 0;
    public static final double kFF = 0;
    public static final int currentLimit = 60;

    // public static final double kS = 0;
    // public static final double kG0 = 40;
    // public static final double kV = 8;

    // public static final double kG1 = 40;
    // public static final double kG2 = 40;

    public static final double kPSlot1 = 0;
    public static final double kISlot1 = 0;
    public static final double kDSlot1 = 0;

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

    public static final Slot1Configs elevatorGains1 =
        new Slot1Configs()
            .withKP(kP)
            .withKI(kI)
            .withKD(kD)
            .withKS(kS)
            .withKV(kV)
            .withKA(kA)
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(kG1);

    public static final Slot2Configs elevatorGains2 =
        new Slot2Configs()
            .withKP(kP)
            .withKI(kI)
            .withKD(kD)
            .withKS(kS)
            .withKV(kV)
            .withKA(kA)
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(kG2);

    public static final TalonFXConfiguration elevatorConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(currentLimit))
                    .withStatorCurrentLimitEnable(true));

    public static final SysIdRoutine.Config sysIdConfig =
        new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(4), Seconds.of(25));
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
