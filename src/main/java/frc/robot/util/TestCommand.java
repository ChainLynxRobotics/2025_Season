public class TestCommand extends Command {
  @Override
  public boolean isFinished() {
    return !DriverStation.getInstance().isTest();
  }
}
