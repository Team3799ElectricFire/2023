// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Lights;

public class Lights_SetMode extends CommandBase {
  private Lights _lights;
  private int _mode;
  /** Creates a new Lights_SetMode. */
  public Lights_SetMode(Lights lights, int mode) {
    // Use addRequirements() here to declare subsystem dependencies.
    _lights = lights;
    _mode = mode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _lights.setMode(_mode);
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
    return true;
  }
}
