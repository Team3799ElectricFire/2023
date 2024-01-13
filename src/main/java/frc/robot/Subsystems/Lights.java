// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lights extends SubsystemBase {
  DigitalOutput Pin0, Pin1, Pin2;

  /** Creates a new Lights. */
  public Lights() {
    Pin0 = new DigitalOutput(Constants.LightsPin0);
    Pin1 = new DigitalOutput(Constants.LightsPin1);
    Pin2 = new DigitalOutput(Constants.LightsPin2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMode(int newMode) {
    if (newMode >= 0 && newMode <= 7) {
      Pin0.set((newMode & 0x01) != 0);
      Pin1.set((newMode & 0x02) != 0);
      Pin2.set((newMode & 0x04) != 0);
    }
  }
}
