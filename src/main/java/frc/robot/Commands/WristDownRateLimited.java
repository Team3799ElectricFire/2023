// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class WristDownRateLimited extends CommandBase {
  private Arm _arm;
  /** Creates a new WristDownRateLimited. */
  public WristDownRateLimited(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    _arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Start Wrist Down");
    _arm.wristBrakeOFF();
    _arm.wristResetPid();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _arm.rateLimitedWristJog(-1.0);
    _arm.wristRunPid();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("End Wrist Down");
    _arm.wristBrakeON();
    _arm.wristStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
