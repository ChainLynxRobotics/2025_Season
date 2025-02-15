package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.Config;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;

public class Elevator extends SubsystemBase {
  private SparkMax sparkMaxLeader = new SparkMax(ElevatorConstants.sparkMaxFollowerCANId, MotorType.kBrushless);
  private SparkMax sparkMaxFollower = new SparkMax(ElevatorConstants.sparkMaxCANId, MotorType.kBrushless);
  private SparkAbsoluteEncoder encoder = sparkMaxLeader.getAbsoluteEncoder();
  private ClosedLoopSlot slot = ClosedLoopSlot.kSlot0;


  private Distance setpoint = Meters.of(0);

  public SparkClosedLoopController pidControllerLeader;


  public Elevator() {
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    SparkMaxConfig leaderConfig = new SparkMaxConfig();

    globalConfig.idleMode(Config.idleMode);

    globalConfig.
    encoder.positionConversionFactor(Config.positionConvertionFactor)
    .velocityConversionFactor(Config.velocityConvertionFactor);

    globalConfig.
      closedLoop.feedbackSensor(Config.feedbackSensor)
        .pidf(Config.pidP, Config.pidI, Config.pidD, Config.feedForward)
        .iZone(Config.pidIZone);

    leaderConfig.apply(globalConfig).inverted(Config.inverted);

    followerConfig.apply(globalConfig).follow(sparkMaxLeader);

    pidControllerLeader = sparkMaxLeader.getClosedLoopController();
    
    sparkMaxLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    sparkMaxFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setSetPoint(Distance setpoint) {
    this.setpoint = setpoint;
  }

  @Override
  public void periodic() {
    pidControllerLeader.setReference(
    (setpoint.in(Meters) / ElevatorConstants.maxHight.in(Meters)), ControlType.kPosition, slot);
  }

  public Distance getElevatorHight() {
    return Meters.of(encoder.getPosition()).minus(ElevatorConstants.encoderOffset);
  }
}
