package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.ClimberConstants.*;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;

public class ClimberIOSpark implements ClimberIO {
  private SparkMax motorController;
  private SparkAbsoluteEncoder encoder;
  private SparkClosedLoopController controller;

  public ClimberIOSpark(SparkMax motorController) {
    this.motorController = motorController;
    var config = new SparkMaxConfig();
    this.encoder = motorController.getAbsoluteEncoder();
    this.controller = motorController.getClosedLoopController();

    // Todo: test what the minium current limit is
    config.inverted(false).smartCurrentLimit(40).secondaryCurrentLimit(50);
    config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(kP, kI, kD);
    motorController.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ClimberIOInputsAutoLogged inputs) {
    inputs.climberAngle = Rotations.of(encoder.getPosition());
    inputs.motorOutput = motorController.getAppliedOutput();
  }

  @Override
  public void setAngle(Angle angle) {
    controller.setReference(angle.in(Rotations), ControlType.kPosition);
  }

  @Override
  public void setOutput(double output) {
    motorController.set(output);
  }

  @Override
  public void stopMotor() {
    motorController.stopMotor();
  }

  @Override
  public void close() {
    motorController.close();
  }
}
