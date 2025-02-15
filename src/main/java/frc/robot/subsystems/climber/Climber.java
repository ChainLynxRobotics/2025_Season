package frc.robot.subsystems.climber;


import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Climber extends SubsystemBase {

  private SparkMax motorController;
  private DigitalInput m_limSwitchUpper;
  private DigitalInput m_limSwitchLower;

  public Climber() {
    motorController = new SparkMax(ClimberConstants.kClimberMotorID, MotorType.kBrushless);
    
    motorController.set(0);

    m_limSwitchUpper = new DigitalInput(ClimberConstants.kUpperLimSwitchId);
    m_limSwitchLower = new DigitalInput(ClimberConstants.kUpperLimSwitchId);

    motorController.getEncoder().setPosition(0);
    
    SmartDashboard.putNumber("climber encoder rots", motorController.getEncoder().getPosition());
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("lim motor up?", m_limSwitchUpper.get());
    SmartDashboard.putBoolean("lim motor low?", m_limSwitchLower.get());
    SmartDashboard.putNumber("climber encoder rots", motorController.getEncoder().getPosition());
}
  public double getmotorEncoderPosition() {
    return motorController.getEncoder().getPosition();
  }
  public void setMotor(boolean reverse) {
    int dir = reverse ? -1 : 1;
    motorController.set(ClimberConstants.kClimberMotorMult * dir);
  }

  public void stopMotor() {
    motorController.stopMotor();
  }

  public SparkMax getmotor() {
    return motorController;
  }
  // neutral signal SHOULD then default to brake mode
  public void disable() {
    motorController.disable();
  }

  public boolean getLimitUpperSwitchValue() {
    if(!m_limSwitchUpper.get()) {
        return true;
    }
    return false;
  }
  public boolean getLimitLowerSwitchValue() {
    if(!m_limSwitchLower.get()) {
        return true;
    } 
        return false;
  }
  }