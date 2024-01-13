// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class ShoulderToAngle extends CommandBase {
  private Arm _arm;
  private double _targetAngleDegrees;
  private int timeAtGoal = 0;
  /** Creates a new ShoulderToAngle. */
  public ShoulderToAngle(Arm arm, double targetAngleDegrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    _arm = arm;
    _targetAngleDegrees = targetAngleDegrees;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Start Shoulder GoTo");
    _arm.shoulderBrakeOFF();
    _arm.shoulderResetPid();
    _arm.shoulderSetGoal(_targetAngleDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _arm.shoulderRunPid();

    if (_arm.shoulderAtGoal()) {
      timeAtGoal = timeAtGoal + 1;
    } else {
      timeAtGoal = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("End Shoulder GoTo");
    _arm.shoulderBrakeON();
    _arm.shoulderStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timeAtGoal > 5;
  }
}
