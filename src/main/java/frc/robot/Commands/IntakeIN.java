// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Intake;

public class IntakeIN extends CommandBase {
  private final Intake _intake;
  /** Creates a new IntakeIN. */
  public IntakeIN(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    _intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _intake.PickUpGamePiece();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _intake.intakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
