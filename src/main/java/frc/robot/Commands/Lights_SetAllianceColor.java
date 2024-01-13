// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Lights;

public class Lights_SetAllianceColor extends CommandBase {
  private Lights _lights;
  /** Creates a new Lights_SetAllianceColor. */
  public Lights_SetAllianceColor(Lights lights) {
    // Use addRequirements() here to declare subsystem dependencies.
    _lights = lights;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Alliance color = DriverStation.getAlliance();

    if (color == Alliance.Red) {
      _lights.setMode(Constants.LED_red);

    } else if (color == Alliance.Blue) {
      _lights.setMode(Constants.LED_blue);

    } else {
      _lights.setMode(Constants.LED_rainbow);

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
    return true;
  }
}
