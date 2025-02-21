package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class TestCommand extends Command {
  @Override
  public boolean isFinished() {
    return !DriverStation.isTest();
  }
}
