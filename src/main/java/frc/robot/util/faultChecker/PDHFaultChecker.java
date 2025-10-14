package frc.robot.util.faultChecker;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.Constants.PDHConstants;

public class PDHFaultChecker extends AbstractFaultChecker {
  private PowerDistributionFaults faults;
  private PowerDistribution robotPower;

  public PDHFaultChecker(String componentName) {
    super(componentName);
  }

  public PDHFaultChecker(PowerDistribution robotPower, String componentName) {
    super(componentName);
    this.robotPower = robotPower;
    super.addFaults(PDHConstants.getPDHFaults(() -> (faults), robotPower));
  }

  public void updateFaults() {
    faults = robotPower.getFaults();
    super.updateFaults();
  }
}
