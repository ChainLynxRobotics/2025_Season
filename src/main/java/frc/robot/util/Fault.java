package frc.robot.util;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.util.datalog.StringLogEntry;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import java.util.Objects;
import java.util.function.Function;

/**
 * Encodes the status of a fault as well as the required resorces to update and notify the driver
 */
public class Fault {
  String faultName;
  Function<Boolean, StatusSignal<Boolean>> functionToCheckFault;
  NotificationLevel level = NotificationLevel.WARNING;
  boolean hasFault;
  boolean hadFault;

  private record FaultInfo(String header, String payload) {
    public FaultInfo {
      Objects.requireNonNull(header);
      Objects.requireNonNull(payload);
    }
  }
  /**
   * @param functionToCheckFault function that indicates whether a particular fault has occurred
   *     (e.g. TalonFX.getFault_DeviceTemp indicated whether a talon is overheated)
   */
  public Fault(Function<Boolean, StatusSignal<Boolean>> functionToCheckFault) {
    this.functionToCheckFault = functionToCheckFault;
    // Extracts the fault name from the name of the function
    // getFault_hardware -> hardware
    faultName = functionToCheckFault.toString().replace("getFault_", "");
  }

  /**
   * @param functionToCheckFault function that indicates whether a particular fault has occurred
   *     (e.g. TalonFX.getFault_DeviceTemp indicated whether a talon is overheated)
   * @param name the name of the fault
   */
  public Fault(Function<Boolean, StatusSignal<Boolean>> functionToCheckFault, String name) {
    this.functionToCheckFault = functionToCheckFault;
    this.faultName = name;
  }
  /**
   * @param functionToCheckFault function that indicates whether a particular fault has occurred
   *     (e.g. TalonFX.getFault_DeviceTemp indicated whether a talon is overheated)
   * @param notificationLevel the sevarity of the fault (see {@link NotificationLevel}).
   *     NotificationLevel.INFO is ignored
   */
  public Fault(
      Function<Boolean, StatusSignal<Boolean>> functionToCheckFault,
      NotificationLevel notificationLevel) {
    this.functionToCheckFault = functionToCheckFault;
    this.faultName = functionToCheckFault.toString().replace("getFault_", "");
    this.level = notificationLevel;
  }
  /**
   * @param functionToCheckFault function that indicates whether a particular fault has occurred
   *     (e.g. TalonFX.getFault_DeviceTemp indicated whether a talon is overheated)
   * @param notificationLevel the sevarity of the fault (see {@link NotificationLevel}).
   *     NotificationLevel.INFO is ignored
   * @param name the name of the fault
   */
  public Fault(
      Function<Boolean, StatusSignal<Boolean>> functionToCheckFault,
      NotificationLevel notificationLevel,
      String name) {
    this.functionToCheckFault = functionToCheckFault;
    this.faultName = name;
    this.level = notificationLevel;
  }

  /** Updates whether the fault is active and the state in the last call */
  public void updateFault() {
    if (functionToCheckFault.apply(true).getValue()) {
      System.out.println("Fault detected");
    }
    hadFault = hasFault;
    hasFault = functionToCheckFault.apply(true).getValue();
  }

  private FaultInfo getFaultString(String subsystemName) {
    return new FaultInfo(
        subsystemName + "fault", faultName + " fault" + (hasFault ? "" : " resolved"));
  }

  /**
   * @param componentName the name of the commponent that has the fault
   */
  public void sendNotification(String componentName) {
    FaultInfo faultInfo = getFaultString(componentName);
    Elastic.sendNotification(new Notification(level, faultInfo.header, faultInfo.payload));
  }
  /**
   * @param componentName the name of the commponent that has the fault
   * @param logEntry the log to add the fault to
   */
  public void logFault(String componentName, StringLogEntry logEntry) {
    FaultInfo faultString = getFaultString(componentName);
    logEntry.append(faultString.header + ". " + faultString.payload);
  }
}
