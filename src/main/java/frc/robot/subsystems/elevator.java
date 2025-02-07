package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.Config;

public class elevator extends SubsystemBase {
  private SparkMax sparkMax = new SparkMax(ElevatorConstants.sparkMaxCANId, MotorType.kBrushless);

  private Distance setPoint = Meters.of(0);

  private SparkClosedLoopController pidController;

  public elevator() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(Config.inverted).idleMode(Config.idleMode);
    config
        .encoder
        .positionConversionFactor(Config.positionConvertionFactor)
        .velocityConversionFactor(Config.velocityConvertionFactor);
    config
        .closedLoop
        .feedbackSensor(Config.feedbackSensor)
        .pidf(Config.pidP, Config.pidI, Config.pidD, Config.feedForward)
        .iZone(Config.pidIZone);

    pidController = sparkMax.getClosedLoopController();
  }

  public void setSetPoint(Distance setpoint) {
    this.setPoint = setpoint;
  }

  @Override
  public void periodic() {
    pidController.setReference(
        (setPoint.in(Meters) / ElevatorConstants.maxHight.in(Meters)), ControlType.kPosition);
  }

  public void resetEncoder() {
    sparkMax.getEncoder().setPosition(0);
  }

  public Distance getElevatorHight() {
    return Meters.of(sparkMax.getEncoder().getPosition());
  }
}
