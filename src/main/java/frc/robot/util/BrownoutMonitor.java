package frc.robot.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PDHConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.util.faultChecker.PDHFaultChecker;

public class BrownoutMonitor extends SubsystemBase {
  private PowerDistribution robotPower;
  private Drive drive;
  private Climber climber;
  private Elevator elevator;
  private Manipulator manipulator;

  private PDHFaultChecker faultChecker;

  public BrownoutMonitor(Drive drive, Climber climber, Elevator elevator, Manipulator manipulator) {
    this.robotPower = new PowerDistribution(1, ModuleType.kRev);
    this.drive = drive;
    this.climber = climber;
    this.elevator = elevator;
    this.manipulator = manipulator;

    faultChecker = new PDHFaultChecker(robotPower, "PDH");
  }

  @Override
  public void periodic() {
    DogLog.log("pdh voltage", robotPower.getVoltage());
    DogLog.log("pdh current", robotPower.getAllCurrents());

    faultChecker.updateFaults();

    boolean shutdown = false;
    if (Amps.of(robotPower.getTotalCurrent()).gte(PDHConstants.kMaxTotalCurrentDraw)) {
      shutdown = true;
    }

    if (Volts.of(robotPower.getVoltage()).gte(PDHConstants.kMaxInputVoltage)) {
      shutdown = true;
    }

    if (Celsius.of(robotPower.getTemperature()).gte(PDHConstants.kMaxTemperature)) {
      shutdown = true;
    }

    if (shutdown) {
      drive.stop();
      climber.stop();
      elevator.stop();
      manipulator.stop();
    }

    // TODO stop specific subsystems for specific PDH channels (0-19 high, 20-23 low)
  }
}
