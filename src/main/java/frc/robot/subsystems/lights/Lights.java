package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightConstants.LightsState;
import frc.robot.Constants.LightConstants;

public class Lights extends SubsystemBase {
  private AddressableLED ledStrip;
  private AddressableLEDBuffer ledBuffer;
  private LightsState prevState;
  private LightsState curState;

  public Lights() {
    this.ledStrip = new AddressableLED(LightConstants.pwmPort);
    this.ledBuffer = new AddressableLEDBuffer(LightConstants.numLeds);
    this.prevState = LightsState.STOPPED;
    this.curState = LightsState.IDLE;

    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  @Override
  public void periodic() {
    if (curState != prevState) {
      setLedState();
      prevState = curState;
    }
  }

  public void setLedState() {
    LEDPattern pattern = LightConstants.ledMap.get(curState);
    pattern.applyTo(ledBuffer);
  }

  public void updateState(LightsState state) {
    this.curState = state;
  }
}
