package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public final class GetAliance {
  public static Alliance getAlliance() {
    if (DriverStation.getAlliance().isPresent()) {
      return DriverStation.getAlliance().get();
    }
    return Constants.defaultAlliance;
  }

  public static boolean getAllianceBoolean() {
    return getAlliance() == Alliance.Blue;
  }
}
