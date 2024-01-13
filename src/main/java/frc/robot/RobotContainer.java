// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.*;
import frc.robot.Subsystems.*;

public class RobotContainer {
  // Sendable chooser that appears on driverstation for the drivers to pick the autonomous mode
  public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  // Subsystems
  private Drivetrain drivetrain = new Drivetrain();
  private Intake intake = new Intake();
  private Arm arm = new Arm();
  private Lights lights = new Lights();

  // Driverstation (controller)
  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController copilot = new CommandXboxController(1);

  // Constructor, runs when robot first turns on
  public RobotContainer() {
    configureBindings();
    createAutoOptions();
  }

  private void configureBindings() {
    // Default commands for subsystems that have them
    drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, driver::getLeftY, driver::getLeftX, driver::getRightX));
    arm.setDefaultCommand(new Arm_CopilotSticks(arm, copilot::getLeftY, copilot::getRightY));

    // Driver controller
    driver.back().onTrue(new InstantCommand(drivetrain::toggleDriveRobotRelative, drivetrain));
    driver.start().onTrue(new InstantCommand(drivetrain::zeroHeading));
    driver.rightBumper().whileTrue(new IntakeIN(intake));
    driver.rightTrigger().whileTrue(new IntakeOUT(intake));
    driver.leftBumper().onTrue(new InstantCommand(intake::ToggleJaw));
    driver.povUp().whileTrue(new WristUpRateLimited(arm));
    driver.povDown().whileTrue(new WristDownRateLimited(arm));
    driver.y().whileTrue(new ShoulderUpRateLimited(arm));
    driver.a().whileTrue(new ShoulderDownRateLimited(arm));
    driver.leftStick().onTrue(new InstantCommand(drivetrain::setHighSpeed, drivetrain));
    driver.rightStick().onTrue(new InstantCommand(drivetrain::setLowSpeed, drivetrain));
    driver.leftTrigger().onTrue(new InstantCommand(drivetrain::setTurboSpeed, drivetrain));

    // Copilot controller
    copilot.rightBumper().whileTrue(new IntakeIN(intake));
    copilot.rightTrigger().whileTrue(new IntakeOUT(intake));
    copilot.leftBumper().onTrue(new InstantCommand(intake::ToggleJaw));
    copilot.x().onTrue(new SequentialCommandGroup(
      new WristToAngle(arm, Constants.WristHomeAngleDegrees),
      new ShoulderToAngle(arm, Constants.ShoulderHomeAngleDegrees)
    ));
    copilot.a().onTrue(new SequentialCommandGroup(
      new WristToAngle(arm, Constants.WristHomeAngleDegrees),
      new ShoulderToAngle(arm, Constants.ShoulderFloorAngleDegrees),
      new WristToAngle(arm, Constants.WristFloorAngleDegrees)
    ));
    copilot.b().onTrue(new SequentialCommandGroup(
      new WristToAngle(arm, Constants.WristHomeAngleDegrees),
      new ShoulderToAngle(arm, Constants.ShoulderMidAngleDegrees),
      new WristToAngle(arm, Constants.WristScoringAngleDegrees)
    ));
    copilot.y().onTrue(new SequentialCommandGroup(
      new WristToAngle(arm, Constants.WristHomeAngleDegrees),
      new ShoulderToAngle(arm, Constants.ShoulderHighAngleDegrees),
      new WristToAngle(arm, Constants.WristScoringAngleDegrees)
    ));
    copilot.povUp().onTrue(new Lights_SetMode(lights, Constants.LED_yellow));
    copilot.povLeft().onTrue(new Lights_SetMode(lights, Constants.LED_rainbow));
    copilot.povDown().onTrue(new Lights_SetMode(lights, Constants.LED_purple));
    copilot.povRight().onTrue(new Lights_SetAllianceColor(lights));
    
    // driver station commands
    SmartDashboard.putData("Stow Arm", new SequentialCommandGroup(
      new PrintCommand("Start Stow Arm"),
      new WristToAngle(arm, Constants.WristHomeAngleDegrees),
      new ShoulderToAngle(arm, Constants.ShoulderHomeAngleDegrees),
      new WristToAngle(arm, Constants.WristStowAngleDegrees)
    ));
    //SmartDashboard.putData(new InstantCommand(this::createAutoOptions));
  }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**
   * Create the list of autonomous options that gets sent to the driverstation
   */
  private void createAutoOptions() {
    // Always create a "Do Nothing" option
    autoChooser.setDefaultOption("Do Nothing (Face Towards Opp Wall)", new SequentialCommandGroup(
      new Auto_SetHeading(drivetrain, 0.0),
      new InstantCommand(drivetrain::setDriveFieldRelative, drivetrain)
    ));

    // Add other options to the list here
    //autoChooser.addOption(String name, Command commandToRun);
    autoChooser.addOption("Do Nothing (Face Back, OUR Wall)", new SequentialCommandGroup(
      new Auto_SetHeading(drivetrain, 180.0),
      new InstantCommand(drivetrain::setDriveFieldRelative, drivetrain)
    ));

    autoChooser.addOption("Drive Forward 2m (Face Towards Opp Wall)", new SequentialCommandGroup(
      new Auto_SetHeading(drivetrain, 0),
      new InstantCommand(drivetrain::setDriveFieldRelative, drivetrain),
      new Auto_PathPlannerTest(
        drivetrain,
        PathPlanner.loadPath("StraightForward", new PathConstraints(2.0, 1.5)),
        true
      )
    ));

    autoChooser.addOption("Drive Backward 2m (Face Back, OUR Wall)", new SequentialCommandGroup(
      new Auto_SetHeading(drivetrain, 180),
      new InstantCommand(drivetrain::setDriveFieldRelative, drivetrain),
      new Auto_PathPlannerTest(
        drivetrain,
        PathPlanner.loadPath("StraightBackward", new PathConstraints(2.0, 1.5)),
        true
      )
    ));

    autoChooser.addOption("Cube Mid, then back out (Face Back, OUR Wall)", new SequentialCommandGroup(
      new Auto_SetHeading(drivetrain, 180.0),
      new InstantCommand(drivetrain::setDriveFieldRelative, drivetrain),
      new InstantCommand(intake::JawOpen, intake),
      new WristToAngle(arm, Constants.WristHomeAngleDegrees),
      new IntakeOUT(intake).withTimeout(1.0),
      new WaitCommand(2.0),
      new Auto_PathPlannerTest(
        drivetrain,
        PathPlanner.loadPath("BackOutOfCommunity", new PathConstraints(2.0, 1.5)),
        true
      )
    ));

    autoChooser.addOption("Test Balance Routine", new Auto_Balance(drivetrain));

    // Put the chooser on the driverstation
    SmartDashboard.putData(autoChooser);
  }

  public void bumplessTransferArmSetpoints() {
    arm.bumplessTransferGoals();
  }
  public void syncArmEncoders() {
    arm.syncEncoders();
  }
}
 