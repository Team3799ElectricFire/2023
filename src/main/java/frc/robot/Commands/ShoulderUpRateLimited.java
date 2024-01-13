// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class ShoulderUpRateLimited extends CommandBase {
  private Arm _arm;
  /** Creates a new ShoulderUpRateLimited. */
  public ShoulderUpRateLimited(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    _arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Start Shoulder Up");
    _arm.shoulderBrakeOFF();
    _arm.shoulderResetPid();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _arm.rateLimitedShoulderJog(+1.0);
    _arm.shoulderRunPid();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("End Shoulder Up");
    _arm.shoulderBrakeON();
    _arm.shoulderStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
