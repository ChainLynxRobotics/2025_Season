package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.ManipulatorConstants.*;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class Manipulator extends SubsystemBase implements Logged {
  private SparkMaxConfig config;
  private SparkMax pivot;
  private SparkMax rollers;
  private SparkAbsoluteEncoder pivotEncoder;
  private SparkClosedLoopController controller;
  private DigitalInput coralDetect;

  public Manipulator() {
    config = new SparkMaxConfig();
    pivot = new SparkMax(PIVOT_ID, MotorType.kBrushless);
    rollers = new SparkMax(ROLLER_ID, MotorType.kBrushless);
    coralDetect = new DigitalInput(LINE_BREAK_PORT);
    pivotEncoder = pivot.getAbsoluteEncoder();
    controller = pivot.getClosedLoopController();

    config.idleMode(IdleMode.kBrake);
    config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(kP, kI, kD);
    pivot.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    this.log("manipulator/angle", pivotEncoder.getPosition());
    this.log("manipulator/rollers", rollers.get() != 0 ? true : false);
    this.log("manipulator/hasCoral", hasCoral());
  }

  public Command setAngle(double setpoint) {
    return this.run(
        () -> {
          double ff = Math.cos(Rotations.of(pivotEncoder.getPosition()).in(Units.Radians)) * kFF;
          controller.setReference(
              setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kPercentOut);
        });
  }

  public Command spinRollers() {
    return this.run(() -> rollers.set(defaultRollerSpeed)).finallyDo(() -> rollers.set(0.0));
  }

  public boolean hasCoral() {
    return coralDetect.get();
  }
}
