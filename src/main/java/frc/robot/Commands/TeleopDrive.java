// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

public class TeleopDrive extends CommandBase {
  private Drivetrain _drivetrain;
  private DoubleSupplier _xSupplier, _ySupplier, _rotSupplier;
  private SlewRateLimiter _xLimiter = new SlewRateLimiter(Constants.PanRateLimit);
  private SlewRateLimiter _yLimiter = new SlewRateLimiter(Constants.PanRateLimit);
  private SlewRateLimiter _rotLimiter = new SlewRateLimiter(Constants.TurnRateLimit);

  /** Creates a new TeleopDrive. */
  public TeleopDrive(Drivetrain drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this._drivetrain = drivetrain;
    addRequirements(drivetrain);
    this._xSupplier = xSupplier;
    this._ySupplier = ySupplier;
    this._rotSupplier = rotSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xRawDemand = -1 * _xSupplier.getAsDouble(); // positive is fwd
    double yRawDemand = -1 * _ySupplier.getAsDouble(); // positive is left
    double rRawDemand = -1 * _rotSupplier.getAsDouble(); // positive is ccw from above

    double xDemand = _xLimiter.calculate(xRawDemand * Math.abs(xRawDemand));
    double yDemand = _yLimiter.calculate(yRawDemand * Math.abs(yRawDemand));
    double rDemand = _rotLimiter.calculate(rRawDemand * Math.abs(rRawDemand));

    // Calculate magnitude of each stick's offset from center
    double leftMagnatude = Math.sqrt(xRawDemand*xRawDemand + yRawDemand*yRawDemand);
    double rightMagnatude = Math.abs(rRawDemand);

    // Only command the modules to move if the driver input is far enough from center
    if (leftMagnatude > Constants.minThumbstickMagnitude || rightMagnatude > Constants.minThumbstickMagnitude) {
      // Drive
      if (_drivetrain.getDriveRobotRelative()) {
        _drivetrain.driveRobotRelative(xDemand, yDemand, rDemand);
  
      } else {
        _drivetrain.driveFieldRelative(xDemand, yDemand, rDemand);
  
      }
    } else {
      // Stop
      _drivetrain.stop();

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
