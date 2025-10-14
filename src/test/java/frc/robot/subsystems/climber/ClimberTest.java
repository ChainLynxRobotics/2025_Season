package frc.robot.subsystems.climber;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.ClimberConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ClimberTest {
  Climber climber;
  SparkMaxSim motorSim;
  TalonFXSimState sim;

  static final double DELTA = 1e-2;

  @BeforeEach
  void setup() {
    // RobotAsserts.(DELTA, DELTA, DELTA);
    assert HAL.initialize(500, 0);
    // TalonFX talonfx = new TalonFX(0);
    // talonfx.sim sim = new TalonFXSimState(new TalonFX(0));

    SparkMax climberMotor = new SparkMax(ClimberConstants.kClimberMotorID, MotorType.kBrushless);
    this.climber = new Climber(climberMotor);
    this.motorSim = new SparkMaxSim(climberMotor, DCMotor.getNEO(1));
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach
  void shutdown() throws Exception {
    climber.close();
  }

  @Test
  void testRunForwards() {
    assertNotNull(climber);
    climber.run(false);
    assertEquals(ClimberConstants.kClimberMotorMult * -1, motorSim.getSetpoint(), DELTA);
  }

  @Test
  void testRunBackwards() {
    assertNotNull(climber);

    climber.run(true);
    assertEquals(ClimberConstants.kClimberMotorMult, motorSim.getSetpoint(), DELTA);
  }

  /*
  @Test
  void testSetPosition() {
    assertNotNull(climber);

    double position = 1.0;
    motorSim.getAbsoluteEncoderSim().setPosition(position);
    assertEquals(climber.getPosition(), position, DELTA);
  }

  @Test
  void testMoveToSetpoint() {
    assertNotNull(climber);
    climber.moveToSetpoint(ClimberConstants.State.BOTTOM);
    System.out.println(ElevatorConstants.kAngularSetpointTolerance.in(Rotations));
  }
    */
}
