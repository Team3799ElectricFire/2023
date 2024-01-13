// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class Arm_CopilotSticks extends CommandBase {
  private Arm _arm;
  private DoubleSupplier _wristSupplier, _shoulderSupplier;

  /** Creates a new Arm_CopilotSticks. */
  public Arm_CopilotSticks(Arm arm, DoubleSupplier wristSupplier, DoubleSupplier shoulderSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    _arm = arm;
    _wristSupplier = wristSupplier;
    _shoulderSupplier = shoulderSupplier;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Inputs
    double wristDemand = -1*_wristSupplier.getAsDouble();
    double shoulderDemand = -1*_shoulderSupplier.getAsDouble();

    // Wrist
    if (Math.abs(wristDemand) < 0.10 ||
        (wristDemand > 0 && !_arm.getWristToShoulderOK()) ) {
      _arm.wristBrakeON();
      _arm.wristStop();

      _arm.wristResetPid();
    } else {
      _arm.wristBrakeOFF();

      _arm.rateLimitedWristJog(wristDemand);
      _arm.wristRunPid();
    }

    // Shoulder
    if (Math.abs(shoulderDemand) < 0.10 || 
        (shoulderDemand < 0 && !_arm.getWristToShoulderOK())) {
      _arm.shoulderBrakeON();
      _arm.shoulderStop();

      _arm.shoulderResetPid();
    } else {
      _arm.shoulderBrakeOFF();

      _arm.rateLimitedShoulderJog(shoulderDemand);
      _arm.shoulderRunPid();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
