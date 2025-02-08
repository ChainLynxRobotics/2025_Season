package frc.robot.subsystems.climber;

import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Climber extends SubsystemBase {

  private SparkMax motorController;
  private int multiplier;
  private DigitalInput m_limSwitchmotor;
  private DigitalInput m_limSwitchRight;

  public Climber() {
    motorController = new SparkMax(ClimberConstants.kClimberMotorID, MotorType.kBrushless);
    
    motorController.set(0);
    m_limSwitchmotor = new DigitalInput(9);
    m_limSwitchRight = new DigitalInput(8);

    multiplier = 1;

    motorController.getEncoder().setPosition(0);
    
    SmartDashboard.putNumber("climber encoder rots", motorController.getEncoder().getPosition());
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("motor lim switch", m_limSwitchmotor.get());

    SmartDashboard.putNumber("climber encoder rots", motorController.getEncoder().getPosition());
   // if (motorController.getEncoder().getPosition() < 0
     //   || motorController.getEncoder().getPosition() > ClimberConfig.kUpperRotSoftStop) {
      // motorController.set(0);
      // rightController.set(0);
    }

    /*if (!m_limSwitchmotor.get()) {
      motorController.set(0);
    }

    if (!m_limSwitchRight.get()) {
      rightController.set(0);
    }*/
  public void setIdleMode() {
    
  }

  public double getmotorEncoderPosition() {
    return motorController.getEncoder().getPosition();
  }
  public void setMotor(boolean reverse) {
    multiplier = reverse ? -1 : 1;
    motorController.set(0.9 * multiplier);
  }

  public void stopMotor() {
    motorController.set(0);
  }

  public SparkMax getmotor() {
    return motorController;
  }
}