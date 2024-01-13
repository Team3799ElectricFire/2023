// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_PathPlannerTest extends SequentialCommandGroup {
  private Drivetrain _drivetrain;

  /** Creates a new Auto_PathPlannerTest. */
  public Auto_PathPlannerTest(Drivetrain drivetrain, PathPlannerTrajectory traj, boolean isFirstPath) {
    _drivetrain = drivetrain;
    addRequirements(drivetrain);
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> {
        if (isFirstPath) {
          PathPlannerTrajectory.PathPlannerState transformed = PathPlannerTrajectory.transformStateForAlliance(traj.getInitialState(), DriverStation.getAlliance());
          _drivetrain.resetOdometry(new Pose2d(transformed.poseMeters.getTranslation(), transformed.holonomicRotation));
        }
      }),
      new PPSwerveControllerCommand(
        traj, // Trajectory to follow
        _drivetrain::getPose, // Supplier to get robot's pose
        new PIDController(Constants.AutoDriveP,Constants.AutoDriveI,Constants.AutoDriveD), // X direction controller
        new PIDController(Constants.AutoDriveP,Constants.AutoDriveI,Constants.AutoDriveD), // Y direction controller (same gains as X direction)
        new PIDController(Constants.AutoRotP,Constants.AutoRotI,Constants.AutoRotD), // Rotation controller
        _drivetrain::setChassisSpeeds, // Consumer of chassis speeds, sets these to the swerve modules
        true, // Mirror this path depending on alliance color
        _drivetrain // Subsystem this command requires
      )
    );
  }
}
