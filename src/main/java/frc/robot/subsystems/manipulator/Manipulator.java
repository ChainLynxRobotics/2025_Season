package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.ManipulatorConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import dev.doglog.DogLog;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GameState;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.util.RobotMath;
import frc.robot.util.faultChecker.SparkFaultChecker;

public class Manipulator extends SubsystemBase implements AutoCloseable {
  private SparkMax pivot;
  private SparkMax rollers;
  private SparkMaxConfig pivotConfig;
  private SparkMaxConfig rollerConfig;
  private SparkClosedLoopController pivotController;
  private SparkFaultChecker pivotChecker;
  private SparkFaultChecker rollersChecker;
  private AbsoluteEncoder pivotEncoder;
  private GameState curState = GameState.NONE;
  private Angle setpoint = Constants.reefMap.get(GameState.NONE).angle();
  private Angle error = Rotations.zero();
  private boolean atSetpoint = false;
  private DigitalInput beamBreak;
  private boolean beamBroken = false;

  public Manipulator(SparkMax pivot, SparkMax rollers) {
    this.pivot = pivot;
    this.rollers = rollers;
    this.pivotConfig = new SparkMaxConfig();
    this.rollerConfig = new SparkMaxConfig();
    this.pivotEncoder = pivot.getAbsoluteEncoder();
    this.pivotController = pivot.getClosedLoopController();
    this.pivotChecker = new SparkFaultChecker(pivot, "Pivot SparkMax");
    this.rollersChecker = new SparkFaultChecker(rollers, "Rollers SparkMax");
    this.beamBreak = new DigitalInput(kBeamBreakPort);

    pivotConfig.inverted(false).smartCurrentLimit(ManipulatorConstants.currentLimit);
    pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(kP, kI, kD);
    rollerConfig.idleMode(IdleMode.kCoast);

    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rollers.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    DogLog.log("manipulator/angle", getEncoderPosition().in(Units.Rotations));
    DogLog.log("manipulator/rollers appl out", rollers.getAppliedOutput());
    DogLog.log("manipulator/setpoint", setpoint.in(Rotations));
    DogLog.log("manipulator/pivot appl out", pivot.getAppliedOutput());

    this.error = RobotMath.dist(setpoint, getEncoderPosition());
    DogLog.log("manipulator/error", error.in(Rotations));
    DogLog.log("manipulator/at setpoint", atSetpoint);

    this.beamBroken = beamBreak.get();
    DogLog.log("manipulator/has coral", beamBroken);

    // zeroed at 180 degrees
    double ff = Math.sin(Math.PI - getEncoderPosition().in(Units.Radians)) * kFF;
    DogLog.log("manipulator/ff", ff);
    pivotController.setReference(
        setpoint.in(Rotations),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        ff,
        ArbFFUnits.kPercentOut);
    // pivotChecker.updateFaults();
    // rollersChecker.updateFaults();
  }

  public void move(boolean reverse) {
    pivot.set(kManualPivotSpeed * (reverse ? -1 : 1));
  }

  private Angle getAngle(GameState state) {
    return Constants.reefMap.get(state).angle();
  }

  public void setAngle(GameState state) {
    this.curState = state;
    boolean reverse = Constants.reefMap.get(state).angle().gt(setpoint);
    // Speed to counteract inertial of the coral that may be in the manipulator
    rollers.set(kWhilePivotingSpeed * (reverse ? -1 : 1));
    this.setpoint = getAngle(state);
    this.atSetpoint = false;
  }

  private Angle getEncoderPosition() {
    return Rotations.of(Math.abs(pivotEncoder.getPosition()));
  }

  /** Check if pivot is at angle setpoint to some degree of error */
  public boolean atAngle() {
    this.atSetpoint = error.lt(kRotTolerance);
    return atSetpoint;
  }

  /** Spin rollers forward or backward at default speed */
  public void spinRollers(boolean forward) {
    int multiplier = forward == true ? 1 : -1;
    double speed = kDefaultRollerSpeed;
    if (curState == GameState.HUMAN_PLAYER_STATION) {
      speed = kHumanPlayerStationSpeed;
    }
    rollers.set(speed * multiplier);
  }

  public boolean hasCoral() {
    return beamBroken;
  }

  public void stopRollers() {
    rollers.set(0.0);
  }

  public void stop() {
    pivot.stopMotor();
    rollers.stopMotor();
  }

  @Override
  public void close() throws Exception {
    pivot.close();
    rollers.close();
  }
}
