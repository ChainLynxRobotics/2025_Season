package frc.robot.util;

import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkMax;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

public class SparkMaxFaultChecker {
  private SparkMax motor;

  public SparkMaxFaultChecker(SparkMax motor) {
    this.motor = motor;
  }

  public void checkFaults() {
    Faults faults = motor.getFaults();
    if (faults.can) {
      Elastic.sendNotification(
          getNotification(
              NotificationLevel.WARNING,
              "CAN",
              motor.getDeviceId(),
              "There is likely a break in the CAN bus."));
    }
    if (faults.escEeprom) {
      Elastic.sendNotification(
          getNotification(
              NotificationLevel.ERROR,
              "escEeprom",
              motor.getDeviceId(),
              "Configuration data cannot be read/written; settings cannot be retained. Factory reset, update firmware, and/or check power delivery to SparkMax."));
    }
    if (faults.firmware) {
      Elastic.sendNotification(
          getNotification(
              NotificationLevel.ERROR,
              "firmware",
              motor.getDeviceId(),
              "Firmware may be corrupted or outdated."));
    }
    if (faults.gateDriver) {
      Elastic.sendNotification(
          getNotification(
              NotificationLevel.ERROR,
              "gateDriver",
              motor.getDeviceId(),
              "Circuitry may be damaged due to current draw, heat, or hardware failure."));
    }
    if (faults.other) {
      Elastic.sendNotification(
          getNotification(
              NotificationLevel.ERROR,
              "other",
              motor.getDeviceId(),
              "An unknown firmware, electrical, and/or hardware failure has been detected."));
    }
    if (faults.sensor) {
      Elastic.sendNotification(
          getNotification(
              NotificationLevel.ERROR,
              "sensor",
              motor.getDeviceId(),
              "External or internal sensor damage or disconnection."));
    }
    if (faults.temperature) {
      Elastic.sendNotification(
          getNotification(
              NotificationLevel.ERROR,
              "temperature",
              motor.getDeviceId(),
              "SparkMax has exceeded safe operating temperature."));
    }
  }

  private Notification getNotification(
      NotificationLevel level, String errorName, int controllerId, String description) {
    return new Notification(level, errorName + " Error for SparkMax " + controllerId, description);
  }
}
