// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.SparkBase.ControlType;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.units.measure.Distance;

public class ElevatorSetpoint extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator m_subsystem;
  private final int stage;


  public ElevatorSetpoint(Elevator subsystem, int stage) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);
    this.stage = stage;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // how make integer...
    
    switch(stage) {
      case 1:
        m_subsystem.setSetPoint(ElevatorConstants.stageOneSetpoint);
        break;
      case 2:
        m_subsystem.setSetPoint(ElevatorConstants.stageTwoSetpoint);
        break;
      case 3:
        m_subsystem.setSetPoint(ElevatorConstants.stageThreeSetpoint);
        break;
      case 4:
      m_subsystem.setSetPoint(ElevatorConstants.stageFourSetpoint);
        break;

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
