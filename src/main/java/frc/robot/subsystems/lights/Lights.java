package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightConstants;
import frc.robot.Constants.LightConstants.LightsState;

public class Lights extends SubsystemBase {
  private AddressableLED ledStrip;
  private AddressableLEDBuffer ledBuffer;
  private LightsState state = LightsState.STOPPED;

  public Lights() {
    this.ledStrip = new AddressableLED(LightConstants.pwmPort);
    this.ledBuffer = new AddressableLEDBuffer(LightConstants.numLeds);

    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();

    updateState(state);
  }

  @Override
  public void periodic() {}

  public void updateState(LightsState state) {
    this.state = state;

    LEDPattern pattern = LightConstants.ledMap.get(state);
    pattern.applyTo(ledBuffer);
  }
}
