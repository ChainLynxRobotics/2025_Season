package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveModule;
import frc.robot.util.TestCommand;

public class DriveTests extends TestCommand {

  private Drive drive;
  private DriveModule driveModule;
  private double speed;
  private double rotation;
  private double timeout;
  private Timer timer;

  public DriveTests(Drive drive, DriveModule driveModule) {
    this.drive = drive;
    this.driveModule = driveModule;
    timer = new Timer();
  }

  @Override
  public void initialize() {
    speed = SmartDashboard.getNumber("DriveTests/ModuleSpeed", 0);
    rotation = SmartDashboard.getNumber("DriveTests/ModuleRotation", 0);
    timeout = SmartDashboard.getNumber("DriveTests/Timeout", 1);
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    drive.runModule(driveModule, speed, rotation);
  }

  @Override
  public boolean isFinished() {
    return timer.get() < timeout || super.isFinished();
  }
}
