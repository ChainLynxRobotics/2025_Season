package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GameState;
import frc.robot.util.RobotMath;
import frc.robot.util.faultChecker.CTREFaultChecker;

public class Elevator extends SubsystemBase implements AutoCloseable {
  private TalonFX talonLeader;
  private TalonFX talonFollower;
  private Distance setpoint = Constants.reefMap.get(GameState.NONE).distance();
  private MotionMagicExpoVoltage profile =
      new MotionMagicExpoVoltage(Rotations.of(0)).withEnableFOC(true);

  private final StatusSignal<Angle> leaderPosition;
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderCurrent;
  private final StatusSignal<Double> leaderSetpoint;
  private final StatusSignal<Double> leaderSetpointError;

  private final StatusSignal<Angle> followerPosition;
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerCurrent;
  private final StatusSignal<Double> followerSetpoint;
  private final StatusSignal<Double> followerSetpointError;

  private final Debouncer leaderConnectedDebounce = new Debouncer(0.5);
  private final Debouncer followerConnectedDebounce = new Debouncer(0.5);

  private CTREFaultChecker leadChecker;
  private CTREFaultChecker followerChecker;

  public Elevator(TalonFX talonLeader, TalonFX talonFollower) {
    this.talonLeader = talonLeader;
    this.talonFollower = talonFollower;
    talonFollower.setControl(new Follower(ElevatorConstants.kTalonLeaderCANId, true));

    var elevatorConfig = ElevatorConstants.elevatorConfig;
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorConfig.Slot0 = ElevatorConstants.elevatorGains0;
    elevatorConfig.Slot1 = ElevatorConstants.elevatorGains1;
    elevatorConfig.Slot2 = ElevatorConstants.elevatorGains2;

    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 12;
    elevatorConfig.MotionMagic.MotionMagicAcceleration = 200;
    elevatorConfig.MotionMagic.MotionMagicJerk = 400;

    tryUntilOk(10, () -> talonLeader.getConfigurator().apply(elevatorConfig, 0.25));

    leaderPosition = talonLeader.getPosition();
    leaderVelocity = talonLeader.getVelocity();
    leaderAppliedVolts = talonLeader.getMotorVoltage();
    leaderCurrent = talonLeader.getStatorCurrent();
    leaderSetpoint = talonLeader.getClosedLoopReference();
    leaderSetpointError = talonLeader.getClosedLoopError();

    followerPosition = talonFollower.getPosition();
    followerVelocity = talonFollower.getVelocity();
    followerAppliedVolts = talonFollower.getMotorVoltage();
    followerCurrent = talonFollower.getStatorCurrent();
    followerSetpoint = talonFollower.getClosedLoopReference();
    followerSetpointError = talonFollower.getClosedLoopError();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderPosition,
        leaderVelocity,
        leaderAppliedVolts,
        leaderCurrent,
        leaderSetpoint,
        leaderSetpointError,
        followerPosition,
        followerVelocity,
        followerAppliedVolts,
        followerCurrent,
        followerSetpoint,
        followerSetpointError);
    ParentDevice.optimizeBusUtilizationForAll(talonLeader, talonFollower);

    this.leadChecker = new CTREFaultChecker(talonLeader, "Elevator leader");
    this.followerChecker = new CTREFaultChecker(talonFollower, "Elevator follower");
  }

  @Override
  public void periodic() {
    DogLog.log("elevator/height (rotations)", leaderPosition.getValueAsDouble());
    DogLog.log("elevator/setpoint (rotations)", heightToAngle(setpoint).in(Rotations));
    DogLog.log("elevator/leader output", talonLeader.get());
    DogLog.log("elevator/follower output", talonFollower.get());

    var leaderStatus =
        BaseStatusSignal.refreshAll(
            leaderPosition,
            leaderVelocity,
            leaderAppliedVolts,
            leaderCurrent,
            leaderSetpoint,
            leaderSetpointError);
    var followerStatus =
        BaseStatusSignal.refreshAll(
            followerPosition,
            followerVelocity,
            followerAppliedVolts,
            followerCurrent,
            leaderSetpoint,
            leaderSetpointError);

    for (int i = 0; i < Constants.elevatorStateMap.length; ++i) {
      if (getElevatorHeight().lt(Constants.elevatorStateMap[i])) {}
    }

    DogLog.log("elevator/leader connected", leaderConnectedDebounce.calculate(leaderStatus.isOK()));
    DogLog.log(
        "elevator/follower connected", followerConnectedDebounce.calculate(followerStatus.isOK()));
    DogLog.log("elevator/leader position (rots)", leaderPosition.getValueAsDouble());
    DogLog.log("elevator/follower position (rots)", followerPosition.getValueAsDouble());
    DogLog.log("elevator/leader velocity (rots per sec)", leaderVelocity.getValueAsDouble());
    DogLog.log("elevator/follower velocity (rots per sec)", followerVelocity.getValueAsDouble());
    DogLog.log("elevator/leader voltage", leaderAppliedVolts.getValueAsDouble());
    DogLog.log("elevator/follower voltage", followerAppliedVolts.getValueAsDouble());
    DogLog.log("elevator/leader current (A)", leaderCurrent.getValueAsDouble());
    DogLog.log("elevator/follower current (A)", followerCurrent.getValueAsDouble());
    DogLog.log(
        "elevator/position error (rots)",
        leaderPosition.getValue().minus(heightToAngle(setpoint)).in(Rotations));
    DogLog.log("elevator/leader setpoint position", leaderSetpoint.getValueAsDouble());
    DogLog.log("elevator/leader setpoint error", leaderSetpointError.getValueAsDouble());
    DogLog.log("elevator/folower setpoint position", followerSetpoint.getValueAsDouble());
    DogLog.log("elevator/folower setpoint error", followerSetpointError.getValueAsDouble());

    // this.leadChecker.updateFaults();
    // this.followerChecker.updateFaults();
  }

  public void setSetpoint(GameState stage) {
    this.setpoint = Constants.reefMap.get(stage).distance();
    talonLeader.setControl(profile.withPosition(heightToAngle(setpoint)));
  }

  public boolean atSetpoint() {
    return RobotMath.abs(leaderPosition.getValue().minus(heightToAngle(setpoint)))
        .lt(ElevatorConstants.kSetpointTolerance);
  }

  public void sysIdLog(SysIdRoutineLog log) {
    log.motor("Elevator motors")
        .voltage(talonLeader.getMotorVoltage().getValue())
        .linearPosition(getElevatorHeight())
        .linearVelocity(getElevatorVelocity());
  }

  public void voltageDrive(Voltage volts) {
    // talonLeader.setControl(new VoltageOut(volts));
  }

  public Distance getElevatorHeight() {
    return angleToHeight(leaderPosition.getValue());
  }

  private LinearVelocity getElevatorVelocity() {
    return RobotMath.castToMoreSpecificUnits(
        RobotMath.applyDimesionalAnalysis(
            ElevatorConstants.kAngularSpan, leaderVelocity.getValue()),
        MetersPerSecond.zero());
  }

  private Distance angleToHeight(Angle angle) {
    return (Distance) ElevatorConstants.kAngularSpan.timesDivisor(angle);
  }

  private Angle heightToAngle(Distance height) {
    return (Angle) ElevatorConstants.kSpanAngle.timesDivisor(height);
  }

  public void zeroElevator() {
    setSetpoint(GameState.NONE);
  }

  public void stop() {
    talonLeader.stopMotor();
  }

  @Override
  public void close() {
    talonLeader.close();
  }
}
