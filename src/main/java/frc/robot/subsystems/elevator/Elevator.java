package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.Config;
import monologue.Logged;
import frc.robot.Constants.ElevatorConstants.Stage;

public class Elevator extends SubsystemBase implements AutoCloseable, Logged {
  private SparkMax sparkMaxLeader =
      new SparkMax(ElevatorConstants.sparkMaxCANId, MotorType.kBrushless);
  private SparkMax sparkMaxFollower =
      new SparkMax(ElevatorConstants.sparkMaxFollowerCANId, MotorType.kBrushless);
  private SparkAbsoluteEncoder encoder = sparkMaxLeader.getAbsoluteEncoder();
  public SparkClosedLoopController pidControllerLeader;

  private Distance setpoint = Meters.of(0);

  public Elevator() {
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
    pidControllerLeader = sparkMaxLeader.getClosedLoopController();

    globalConfig
        .idleMode(Config.idleMode)
        .encoder
        .positionConversionFactor(Config.positionConvertionFactor);
    globalConfig
        .closedLoop
        .feedbackSensor(Config.feedbackSensor)
        .pidf(Config.pidP, Config.pidI, Config.pidD, Config.feedForward);
    leaderConfig.apply(globalConfig).inverted(Config.inverted);
    followerConfig.apply(globalConfig).follow(sparkMaxLeader);
    encoderConfig
        .positionConversionFactor(Config.positionConvertionFactor)
        .setSparkMaxDataPortConfig();

    sparkMaxLeader.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    sparkMaxFollower.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setSetpoint(Stage stage) {
    switch (stage) {
      case STAGE_1:
        this.setpoint = ElevatorConstants.levelOneSetpoint;
        break;
      case STAGE_2:
        this.setpoint = ElevatorConstants.levelTwoSetpoint;
        break;
      case STAGE_3:
        this.setpoint = ElevatorConstants.levelThreeSetpoint;
        break;
      case STAGE_4:
        this.setpoint = ElevatorConstants.levelFourSetpoint;
        break;
      default:
        this.setpoint = Meters.of(0);
    }
    setReference();
  }

  public void move(double speed) {
    sparkMaxLeader.set(speed);
  }

  private void setReference() {
    pidControllerLeader.setReference(
        (setpoint.in(Meters) / ElevatorConstants.maxHeight.in(Meters)), ControlType.kPosition);
  }
  @Override 
  public void periodic() {
    this.log("Elevator hight meters", getElevatorHeight().in(Meters)); 
  }
  @Override
  public void close() {
    sparkMaxFollower.close();
    sparkMaxLeader.close();
  }

  public Distance getElevatorHeight() {
    return Meters.of(encoder.getPosition()).minus(ElevatorConstants.encoderOffset);
  }
}
