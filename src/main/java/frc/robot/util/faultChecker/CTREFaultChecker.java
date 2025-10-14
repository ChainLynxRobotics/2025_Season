package frc.robot.util.faultChecker;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.FaultConstants;

public class CTREFaultChecker extends AbstractFaultChecker {

  /**
   * @param deviceName Name of the component to fault check.
   */
  public CTREFaultChecker(String deviceName) {
    super(deviceName);
  }

  /**
   * @param device Talon to check faults on.
   * @param deviceName Name of the component to fault check.
   */
  public CTREFaultChecker(TalonFX device, String deviceName) {
    super(deviceName);
    super.addFaults(FaultConstants.getTalonFXFaults(device));
  }

  /**
   * @param device Pigeon to check faults on.
   * @param deviceName Name of the component to fault check.
   */
  public CTREFaultChecker(Pigeon2 device, String deviceName) {
    super(deviceName);
    super.addFaults(FaultConstants.getPigeonFaults(device));
  }

  /**
   * @param device CANcoder to check faults on.
   * @param deviceName Name of the component to fault check.
   */
  public CTREFaultChecker(CANcoder device, String deviceName) {
    super(deviceName);
    super.addFaults(FaultConstants.getCANCoderFaults(device));
  }
}
