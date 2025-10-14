package frc.robot.util.faultChecker;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

public abstract class AbstractFaultChecker {
  public List<Fault> warningFaults = new ArrayList<>();
  public List<Fault> errorFaults = new ArrayList<>();

  public String componentName;

  private DataLog log;
  private StringLogEntry stringLog;

  /**
   * @param componentName Name of the component that the fault checkers attached to. creates a fault
   *     checker with componentName.
   */
  public AbstractFaultChecker(String componentName) {
    this.componentName = componentName;
    log = DataLogManager.getLog();
    stringLog = new StringLogEntry(log, componentName);
    DataLogManager.start();
  }

  /**
   * Updates all faults in the fault checker. If any faults have been resolved or stated it will log
   * or report them based on sevarity
   */
  public void updateFaults() {
    for (Fault f : warningFaults) {
      f.updateFault();
      if (f.hasFault != f.hadFault) {
        f.logFault(componentName, stringLog);
      }
    }
    for (Fault f : errorFaults) {
      f.updateFault();
      if (f.hasFault != f.hadFault) {
        f.logFault(componentName, stringLog);
        f.sendNotification(componentName);
      }
    }
  }

  /**
   * @return List of all currently faulted faults.
   */
  public List<Fault> getActiveFaults() {
    return Stream.concat(getActiveWarningFaults().stream(), getActiveErrorFaults().stream())
        .toList();
  }

  /**
   * @return List of all currently faulted warning faults.
   */
  public List<Fault> getActiveWarningFaults() {
    List<Fault> activeFaults = new ArrayList<>();
    for (Fault f : warningFaults) {
      if (f.hasFault) {
        activeFaults.add(f);
      }
    }
    return activeFaults;
  }

  /**
   * @return List of all currently faulted error faults.
   */
  public List<Fault> getActiveErrorFaults() {
    List<Fault> activeFaults = new ArrayList<>();
    for (Fault f : errorFaults) {
      if (f.hasFault) {
        activeFaults.add(f);
      }
    }
    return activeFaults;
  }

  /**
   * @param fault Fault to add to the fault checker.
   */
  public void addFault(Fault fault) {
    if (warningFaults != null && fault.level == NotificationLevel.WARNING) {
      this.warningFaults.add(fault);
    }
    if (errorFaults != null && fault.level == NotificationLevel.ERROR) {
      this.errorFaults.add(fault);
    }
  }
  /**
   * @param faults List of faults to add to the fault checker.
   */
  public void addFaults(List<Fault> faults) {
    for (Fault f : faults) {
      addFault(f);
    }
  }

  /**
   * @return If any warning faults are faulted.
   */
  public boolean hasWarningFault() {
    return getActiveWarningFaults().size() != 0;
  }

  /**
   * @return If any error faults are faulted.
   */
  public boolean hasErrorFault() {
    return getActiveErrorFaults().size() != 0;
  }

  /**
   * @return true if no warnings or errors
   */
  public boolean isHealthy() {
    return (!hasWarningFault()) && !(hasErrorFault());
  }

  /**
   * @return true if no warnings
   */
  public boolean isClean() {
    return !hasWarningFault();
  }

  /**
   * @return true if no errors
   */
  public boolean isAlive() {
    return !hasErrorFault();
  }
}
