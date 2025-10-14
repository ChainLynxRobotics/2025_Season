package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.ManipulatorConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.Mode;

public class ManipulatorIOSparkMax implements ManipulatorIO {
  private final SparkMax pivot;
  protected final SparkMax rollers;
  private SparkMaxConfig pivotConfig;
  private SparkMaxConfig rollerConfig;

  private SparkClosedLoopController pivotController;
  private AbsoluteEncoder pivotEncoder;

  private DigitalInput beamBreak;
  private Debouncer beamDebouncer;

  public ManipulatorIOSparkMax(SparkMax pivot, SparkMax rollers) {
    this.pivot = pivot;
    this.rollers = rollers;
    this.pivotConfig = new SparkMaxConfig();
    this.rollerConfig = new SparkMaxConfig();
    this.pivotEncoder = pivot.getAbsoluteEncoder();
    this.pivotController = pivot.getClosedLoopController();

    this.beamBreak = new DigitalInput(kBeamBreakPort);
    this.beamDebouncer = new Debouncer(0.2);

    var encoderConf = new AbsoluteEncoderConfig().zeroCentered(true).inverted(true);
    pivotConfig.smartCurrentLimit(ManipulatorConstants.currentLimit).inverted(true);
    pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(kP, kI, kD);
    pivotConfig.apply(encoderConf);
    if (Constants.currentMode == Mode.SIM) pivotConfig.closedLoop.pid(kSimP, kSimI, kSimD);
    rollerConfig.idleMode(IdleMode.kCoast);

    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rollers.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {
    inputs.manipulatorAngle = Rotations.of(pivotEncoder.getPosition());
    inputs.rollerOutput = rollers.getAppliedOutput();
    inputs.pivotOutput = pivot.getAppliedOutput();
    inputs.beamBroken = beamDebouncer.calculate(beamBreak.get());
  }

  @Override
  public void setAngle(Angle angle) {
    pivotController.setReference(angle.in(Rotations), ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setRollers(double speed) {
    rollers.set(speed);
  }

  @Override
  public void stop() {
    rollers.stopMotor();
    pivot.stopMotor();
  }

  @Override
  public void close() {
    pivot.close();
    rollers.close();
  }
}
