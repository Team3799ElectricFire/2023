// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

public class Auto_Balance extends CommandBase {
  private Drivetrain _drivetrain;
  private PIDController balancePID = new PIDController(Constants.BalanceP, Constants.BalanceI, Constants.BalanceD);
  private int timeOnTarget = 0;
  /** Creates a new Auto_Balance. */
  public Auto_Balance(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    _drivetrain = drivetrain;
    addRequirements(drivetrain);
    balancePID.setTolerance(1.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPitch = _drivetrain.getPitch();

    // Pitch is signed as: NEG == tipping backwards, should drive fwd
    double pidResult = balancePID.calculate(currentPitch, 0.0);


    ChassisSpeeds speeds = new ChassisSpeeds(pidResult, 0.0, 0.0);
    SwerveModuleState[] desiredStates = Constants.kDriveKinematics.toSwerveModuleStates(speeds);

    //System.out.println(currentPitch + " | " + pidResult + " | " + timeOnTarget);
    _drivetrain.setModuleStates(desiredStates);

    if (balancePID.atSetpoint()) {
      timeOnTarget = timeOnTarget + 1;
    } else {
      timeOnTarget = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _drivetrain.setX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//timeOnTarget > 5;
  }
}
