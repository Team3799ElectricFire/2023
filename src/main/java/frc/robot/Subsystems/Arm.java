// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private TalonFX shoulderMotor, wristMotor;
  private Solenoid shoulderBrake, wristBrake;
  private DutyCycleEncoder shoulderEncoder, wristEncoder;
  private ProfiledPIDController shoulderPID, wristPID;
  private ArmFeedforward shoulderFF, wristFF;
  private double shoulderTalonOffset, wristTalonOffset;
  private DigitalInput wristLimitSwitch, shoulderLimitSwitch;
  
  /** Creates a new Arm. */
  public Arm() {
    // finish constructing objects
    shoulderMotor = new TalonFX(Constants.ShoulderMotorID);
    wristMotor = new TalonFX(Constants.WristMotorID);
    shoulderBrake = new Solenoid(Constants.PcmID, PneumaticsModuleType.REVPH, Constants.ShoulderBrakeCH);
    wristBrake = new Solenoid(Constants.PcmID, PneumaticsModuleType.REVPH, Constants.WristBrakeCH);
    shoulderEncoder = new DutyCycleEncoder(Constants.ShoulderEncoderCH);
    wristEncoder = new DutyCycleEncoder(Constants.WristEncoderCH);
    shoulderPID = new ProfiledPIDController(
      Constants.ShoulderP, Constants.ShoulderI, Constants.ShoulderD, 
      new TrapezoidProfile.Constraints(Constants.ShoulderRateLimitDegPerSec, Constants.ShoulderAccelLimitDegPerSecSquared)
    );
    wristPID = new ProfiledPIDController(
      Constants.WristP, Constants.WristI, Constants.WristD, 
      new TrapezoidProfile.Constraints(Constants.WristRateLimitDegPerSec, Constants.WristAccelLimitDegPerSecSquared)
    );
    shoulderFF = new ArmFeedforward(Constants.ShoulderFs, Constants.ShoulderFg, Constants.ShoulderFv);
    wristFF = new ArmFeedforward(Constants.WristFs, Constants.WristFg, Constants.WristFv);
    
    // initial setup
    shoulderMotor.configFactoryDefault();
    shoulderMotor.configSelectedFeedbackCoefficient(Constants.ArmTalonEncoderCoeff);
    
    wristMotor.configFactoryDefault();
    wristMotor.setInverted(true);
    wristMotor.configSelectedFeedbackCoefficient(Constants.ArmTalonEncoderCoeff);

    shoulderPID.setTolerance(Constants.PidToleranceDegrees);
    wristPID.setTolerance(Constants.PidToleranceDegrees);

    shoulderEncoder.setDistancePerRotation(360.0);
    //shoulderEncoder.setDutyCycleRange(1.0/1025.0, 1024.0/1025.0);
    wristEncoder.setDistancePerRotation(360.0);
    //wristEncoder.setDutyCycleRange(1.0/1025.0, 1024.0/1025.0);

    wristLimitSwitch = new DigitalInput(Constants.WristLimitSwitch);
    shoulderLimitSwitch = new DigitalInput(Constants.shoulderLimitSwitch);

    // reset goals so arm doesn't move
    bumplessTransferGoals();
    syncEncoders();
  }

  @Override
  public void periodic() {
    UpdateDashboard();
  }

  public double getShoulderAngleToFloorDegrees() {
    return shoulderEncoder.getAbsolutePosition()*(-355) + Constants.ShoulderOffset;
  }
  public double getWristAngleToArmDegrees() {
    return wristEncoder.getAbsolutePosition()*(356) + Constants.WristOffset;
  }
  public double getWristAngleToFloorDegrees() {
    return getWristAngleToArmDegrees() + getShoulderAngleToFloorDegrees();
  }
  public double getShoulderTalonAngleDegrees() {
    return shoulderMotor.getSelectedSensorPosition()/100.0 - shoulderTalonOffset;
  }
  public double getWristTalonAngleDegrees() {
    return wristMotor.getSelectedSensorPosition()/100.0 - wristTalonOffset;
  }
  public boolean getWristToShoulderOK() {
    return wristLimitSwitch.get() && 
    getWristAngleToArmDegrees() < Constants.WristToArmSoftUpperLimitDegrees;
  }
  public boolean getShoulderOK(){
    return shoulderLimitSwitch.get();
  }

  public void shoulderBrakeON(){
    shoulderBrake.set(false);
  }
  public void shoulderBrakeOFF(){
    shoulderBrake.set(true);
  }
  public void wristBrakeON(){
    wristBrake.set(false);
  }
  public void wristBrakeOFF(){
    wristBrake.set(true);
  }

  public void shoulderSetGoal(double angle) {
    if (angle >= Constants.ShoulderSoftLowerLimitDegrees && angle <= Constants.ShoulderSoftUpperLimitDegrees) {
      shoulderPID.setGoal(angle);
    }
  }
  public void shoulderRunPid() {
    // get current angle
    double shoulderCurrentAngle = getShoulderAngleToFloorDegrees();

    // calculate motor command with PID
    double shoulderPIDResult = shoulderPID.calculate(shoulderCurrentAngle);

    // calculate feedforward
    double shoulderFFResult = shoulderFF.calculate(Math.toRadians(shoulderPID.getGoal().position),0);
    
    // send command to motor
    shoulderMotor.set(TalonFXControlMode.PercentOutput, shoulderPIDResult + shoulderFFResult);
  }
  public boolean shoulderAtGoal() {
    return shoulderPID.atGoal();
  }
  public void shoulderResetPid() {
    shoulderPID.reset(getShoulderAngleToFloorDegrees());
  }

  public void wristSetGoal(double angle) {
    if (angle >= Constants.WristSoftLowerLimitDegrees && angle <= Constants.WristSoftUpperLimitDegrees) {
      wristPID.setGoal(angle);
    }
  }
  public void wristRunPid() {
    // get current angle
    double wristCurrentAngle = getWristAngleToFloorDegrees();

    // calculate motor command with PID
    double wristPIDResult = wristPID.calculate(wristCurrentAngle);

    // calculate feedforward
    double wristFFResult = wristFF.calculate(Math.toRadians(wristPID.getGoal().position), 0);
    
    // send command to motor
    wristMotor.set(TalonFXControlMode.PercentOutput, wristPIDResult + wristFFResult);
  }
  public boolean wristAtGoal() {
    return wristPID.atGoal();
  }
  public void wristResetPid() {
    wristPID.reset(getWristAngleToFloorDegrees());
  }

  public void rateLimitedShoulderJog(double joystick) {
    double goal = shoulderPID.getGoal().position;
    double change = joystick * Constants.ShoulderRateLimitDegPerSec/50;
    goal = goal + change;

    if (goal > Constants.ShoulderSoftUpperLimitDegrees) {
      goal = Constants.ShoulderSoftUpperLimitDegrees;
    } else if (goal < Constants.ShoulderSoftLowerLimitDegrees) {
      goal = Constants.ShoulderSoftLowerLimitDegrees;
    } else if ((!getWristToShoulderOK() || !getShoulderOK()) && joystick < 0) {
    // } else if (getWristAngleToArmDegrees() >= Constants.WristToArmSoftUpperLimitDegrees && joystick < 0) {
      goal = shoulderPID.getGoal().position;
    }

    shoulderPID.setGoal(goal);
  }
  public void rateLimitedWristJog(double joystick) {
    double goal = wristPID.getGoal().position;
    double change = joystick * Constants.WristRateLimitDegPerSec/50;
    goal = goal + change;

    if (goal > Constants.WristSoftUpperLimitDegrees) {
      goal = Constants.WristSoftUpperLimitDegrees;
    } else if (goal < Constants.WristSoftLowerLimitDegrees) {
      goal = Constants.WristSoftLowerLimitDegrees;
    } else if (!getWristToShoulderOK() && joystick > 0) {
    //} else if (getWristAngleToArmDegrees() >= Constants.WristToArmSoftUpperLimitDegrees && joystick > 0) {
      goal = wristPID.getGoal().position;
    }

    wristPID.setGoal(goal);
  }

  public void bumplessTransferGoals() {
    // Reset PID SPs to current arm position to prevent sudden movement
    shoulderPID.setGoal(getShoulderAngleToFloorDegrees());
    shoulderPID.reset(getShoulderAngleToFloorDegrees());
    wristPID.setGoal(getWristAngleToFloorDegrees());
    wristPID.reset(getWristAngleToFloorDegrees());
  }
  public void syncEncoders() {
    shoulderMotor.setSelectedSensorPosition(getShoulderAngleToFloorDegrees());
    shoulderTalonOffset = shoulderMotor.getSelectedSensorPosition();

    wristMotor.setSelectedSensorPosition(getWristAngleToFloorDegrees());
    wristTalonOffset = wristMotor.getSelectedSensorPosition();
  }

  public void testShoulderFeedForwardOnly(double targetAngleDegrees) {
    double correctedTargetAngleDegrees = targetAngleDegrees - 90 - Constants.ShoulderNeutralAngle;
    double shoulderFFResult = shoulderFF.calculate(Math.toRadians(correctedTargetAngleDegrees),0);
    
    shoulderMotor.set(TalonFXControlMode.PercentOutput, shoulderFFResult);
  }
  public void testWristFeedForwardOnly(double targetAngleDegrees) {
    double wristFFResult = wristFF.calculate(Math.toRadians(targetAngleDegrees), 0);
    
    wristMotor.set(TalonFXControlMode.PercentOutput, wristFFResult);
  }
  public void testShoulderMotor(double demand) {
    System.out.println(getShoulderAngleToFloorDegrees() + "," + demand);

    shoulderMotor.set(TalonFXControlMode.PercentOutput, demand);
  }
  public void shoulderStop() {
    shoulderMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }
  public void wristStop() {
    wristMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }
  
  public void UpdateDashboard() {
    SmartDashboard.putNumber("Wrist Angle", getWristAngleToFloorDegrees());
    SmartDashboard.putNumber("Shoulder Angle", getShoulderAngleToFloorDegrees());
    SmartDashboard.putNumber("Wrist Motor Angle", getWristTalonAngleDegrees());
    SmartDashboard.putNumber("Shoulder Motor Angle", getShoulderTalonAngleDegrees());
    SmartDashboard.putNumber("Wrist TO Arm", getWristAngleToArmDegrees());
    SmartDashboard.putBoolean("Wrist TO Arm OK", getWristToShoulderOK());
    SmartDashboard.putBoolean("Shoulder OK", getShoulderOK());


    //SmartDashboard.putNumber("Wrist Goal", wristPID.getGoal().position);
    //SmartDashboard.putNumber("Shoulder Goal", shoulderPID.getGoal().position);
    //SmartDashboard.putNumber("Wrist Setpoint", wristPID.getSetpoint().position);
    //SmartDashboard.putNumber("Shoulder Setpoint", shoulderPID.getSetpoint().position);
    //SmartDashboard.putBoolean("Wrist at Goal", wristPID.atGoal());
    //SmartDashboard.putBoolean("Shoulder at Goal", shoulderPID.atGoal());
    //SmartDashboard.putBoolean("Wrist st Setpoint", wristPID.atSetpoint());
    //SmartDashboard.putBoolean("Shoulder at Setpoint", shoulderPID.atSetpoint());
  }
}