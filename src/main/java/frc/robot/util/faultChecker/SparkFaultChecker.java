package frc.robot.util.faultChecker;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;
import frc.robot.Constants;
import java.util.function.Supplier;

public class SparkFaultChecker extends AbstractFaultChecker {
  private Supplier<Faults> faultSupplier;
  private Supplier<Warnings> warningSupplier;
  public Faults faults;
  public Warnings warnings;

  /**
   * @param faultSupplier Supplier or faults for the device.
   * @param deviceName Name of the device to check faults on.
   */
  public SparkFaultChecker(
      Supplier<Faults> faultSupplier, Supplier<Warnings> warningsSupplier, String deviceName) {
    super(deviceName);
    this.faultSupplier = faultSupplier;
    this.warningSupplier = warningsSupplier;
  }

  /**
   * @param device SparkMax or SparkFlex to check faults on.
   * @param deviceName Name of the device to check faults on.
   */
  public <U extends SparkBase> SparkFaultChecker(U device, String deviceName) {
    super(deviceName);
    this.faultSupplier = () -> (device.getFaults());
    this.warningSupplier = () -> (device.getWarnings());
    super.addFaults(Constants.FaultConstants.getSparkFaults(() -> (faults), () -> (warnings)));
  }

  /**
   * Updates all faults in the fault checker. If any faults have been resolved or stated it will log
   * or report them based on sevarity
   */
  public void updateFaults() {
    faults = faultSupplier.get();
    warnings = warningSupplier.get();
    super.updateFaults();
  }
}
