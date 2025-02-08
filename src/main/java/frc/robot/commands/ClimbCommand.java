// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.*;

public class ClimbCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber m_climber;
  private final boolean reverse;

  public ClimbCommand(Climber climber, boolean reverse) {
    m_climber = climber;
    this.reverse = reverse;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.setMotor(reverse);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_climber.getLimitUpperSwitchValue() || m_climber.getLimitLowerSwitchValue()){
      return true;
    }
  return false;
}
}
