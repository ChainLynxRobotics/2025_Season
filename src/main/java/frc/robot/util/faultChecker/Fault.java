package frc.robot.util.faultChecker;

import edu.wpi.first.util.datalog.StringLogEntry;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import java.util.Objects;
import java.util.function.Supplier;

/**
 * Encodes the status of a fault as well as the required resources to update and notify the driver
 */
public class Fault {
  String faultName;
  Supplier<Boolean> supplierToCheckFault;
  NotificationLevel level = NotificationLevel.ERROR;
  boolean hasFault;
  boolean hadFault;

  /** A recoard that stores the description of a fault as well as its status */
  private record FaultInfo(String header, String payload) {
    public FaultInfo {
      Objects.requireNonNull(header);
      Objects.requireNonNull(payload);
    }
  }

  /**
   * @param functionToCheckFault function that indicates whether a particular fault has occurred
   *     (e.g. TalonFX.getFault_DeviceTemp indicated whether a talon is overheated)
   * @param measage the name of the fault
   */
  public Fault(Supplier<Boolean> supplierToCheckFault, String measage) {
    this.supplierToCheckFault = supplierToCheckFault;
    this.faultName = measage;
  }

  /**
   * @param functionToCheckFault function that indicates whether a particular fault has occurred
   *     (e.g. TalonFX.getFault_DeviceTemp indicated whether a talon is overheated)
   * @param notificationLevel the sevarity of the fault (see {@link NotificationLevel}).
   *     NotificationLevel.INFO is ignored
   * @param measage the name of the fault
   */
  public Fault(
      Supplier<Boolean> supplierToCheckFault, NotificationLevel notificationLevel, String measage) {
    this.supplierToCheckFault = supplierToCheckFault;
    this.faultName = measage;
    this.level = notificationLevel;
  }

  /** Updates whether the fault is active and the state in the last call */
  public void updateFault() {
    hadFault = hasFault;
    hasFault = supplierToCheckFault.get();
  }
  /**
   * @param componentName name of the component that the fault is from
   * @return
   */
  private FaultInfo getFaultString(String componentName) {
    return new FaultInfo(componentName, faultName + (hasFault ? "." : " resolved."));
  }

  /**
   * Sends a elastic notification to alert the drivers to the fault
   *
   * @param componentName the name of the component that has the fault
   */
  public void sendNotification(String componentName) {
    FaultInfo faultInfo = getFaultString(componentName);
    Elastic.sendNotification(new Notification(level, faultInfo.header, faultInfo.payload));
  }
  /**
   * Adds a entry of the fault to a log.
   *
   * @param logEntry the log to add the fault to
   * @param componentName the name of the component that has the fault
   */
  public void logFault(String componentName, StringLogEntry logEntry) {
    FaultInfo faultString = getFaultString(componentName);
    logEntry.append(faultString.header + ". " + faultString.payload);
  }
}
