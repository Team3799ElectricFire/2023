// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax IntakeMotor;
  private Solenoid JawSolenoid = new Solenoid(Constants.PcmID, PneumaticsModuleType.REVPH, Constants.IntakeJawCH);

  /** Creates a new Intake. */
  public Intake() {
    IntakeMotor = new CANSparkMax(Constants.IntakeMotorID, MotorType.kBrushless);

    IntakeMotor.restoreFactoryDefaults();

    IntakeMotor.setIdleMode(IdleMode.kBrake);

    IntakeMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void JawOpen() {
    JawSolenoid.set(true);
  }

  public void JawClose() {
    JawSolenoid.set(false);
  }

  public void ToggleJaw() {
    JawSolenoid.toggle();
  }

  public void PickUpGamePiece() {
    IntakeMotor.set(Constants.IntakeMotorSpeed);
  }

  public void ReleaseGamePiece() {
    IntakeMotor.set(-1.0);
  }
  public void intakeStop(){
    IntakeMotor.set(0);
  }
}
