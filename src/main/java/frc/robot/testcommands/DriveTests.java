import frc.robot.util.TestCommand;
import edu.wpi.first.wpilibj.Timer;

public class DriveTests extends TestCommand {

  private Drivetrain drivetrain;
  private DriveModule driveModule;
  private double speed;
  private double rotation;
  private double timeout;
  private Timer timer;

  public DriveTests(Drivetrain drivetrain, DriveModule driveModule) {
    this.drivetrain = drivetrain;
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
    drivetrain.runModule(driveModule, speed, rotation);
  }

  @Override
  public boolean isFinished() {
    return timer.get() < timeout || super.isFinished();
  }
}
