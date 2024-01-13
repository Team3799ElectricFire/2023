package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    // CAN bus IDs
    public static final int FrontRightSteerMotorID = 2;
    public static final int FrontLeftSteerMotorID = 4;
    public static final int BackRightSteerMotorID = 6;
    public static final int BackLeftSteerMotorID = 8;
    public static final int FrontRightDriveMotorID = 1;
    public static final int FrontLeftDriveMotorID = 3;
    public static final int BackRightDriveMotorID = 5;
    public static final int BackLeftDriveMotorID = 7;
    public static final int ShoulderMotorID = 9; 
    public static final int WristMotorID = 10;
    public static final int IntakeMotorID = 11;
    public static final int PidgeonID = 12;
    public static final int PcmID = 13; 
    public static final int PowerDistributionHub = 14;

    // DIO channels
    public static final int ShoulderEncoderCH = 1;
    public static final int WristEncoderCH = 0;
    public static final int LightsPin0 = 2;
    public static final int LightsPin1 = 3;
    public static final int LightsPin2 = 4;
    public static final int WristLimitSwitch = 5;
    public static final int shoulderLimitSwitch = 6;
    
    // Pneumatic Hub channels
    public static final int IntakeJawCH = 7;
    public static final int ShoulderBrakeCH = 5;
    public static final int WristBrakeCH = 6;

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(22.5);
    public static final double kWheelBase = Units.inchesToMeters(29.5);
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(13.51);
    public static final double kMinSpeedMetersPerSecond = 0.01;
    public static final double kMaxAccelMetersPerSecondSquared = 2.0;
    public static final double kMaxAngularSpeed = 6.0; //kMaxSpeedMetersPerSecond / Units.inchesToMeters(18.55);
    public static final double minThumbstickMagnitude = 0.1;
    public static final Translation2d FrontRightLocation = new Translation2d(+kWheelBase / 2, -kTrackWidth / 2); // Positive X is fwd
    public static final Translation2d FrontLeftLocation = new Translation2d(+kWheelBase / 2, +kTrackWidth / 2); // Positive Y if left
    public static final Translation2d BackRightLocation = new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);
    public static final Translation2d BackLeftLocation = new Translation2d(-kWheelBase / 2, +kTrackWidth / 2);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        FrontRightLocation,
        FrontLeftLocation,
        BackRightLocation,
        BackLeftLocation
    );
    
    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double NeoMotorFreeSpeedRpm = 5676;
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumfrenceMeters = kWheelDiameterMeters * Math.PI; 
    public static final double kDrivingMotorReduction = 5.5;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumfrenceMeters) / kDrivingMotorReduction; 

    public static final double kDrivingEncoderPositionFactor = kWheelCircumfrenceMeters / kDrivingMotorReduction;
    public static final double kDrivingEncoderVelocityFactor = kWheelCircumfrenceMeters / kDrivingMotorReduction / 60.0;
    
    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    // LED modes
    public static final int LED_rainbow = 0;
    public static final int LED_yellow = 2;
    public static final int LED_purple = 5;
    public static final int LED_red = 7;
    public static final int LED_blue = 4;

    // Drivetrain PID Parameters
    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 3;
    public static final double kTurningI = 0.001;
    public static final double kTurningD = 5;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps

    // Arm PID gains
    public static final double ShoulderP = 0.03;
    public static final double ShoulderI = 0.005;
    public static final double ShoulderD = 0;
    public static final double ShoulderFg = 0.157;
    public static final double ShoulderFs = 0;
    public static final double ShoulderFv = 0;

    public static final double WristP = 0.04;
    public static final double WristI = 0;
    public static final double WristD = 0;
    public static final double WristFg = 0.05;
    public static final double WristFs = 0;
    public static final double WristFv = 0;

    public static final double PidToleranceDegrees = 2.0;

    // Arm encoder offsets
    public static final double ShoulderOffset = 16.4;
    public static final double WristOffset = -167.0;

    // Arm encoder positions (all encoder positions are relative to 0 degrees = horizontal)
    public static final double ShoulderSoftLowerLimitDegrees = -80;
    public static final double ShoulderHomeAngleDegrees = -79;
    public static final double ShoulderFloorAngleDegrees = -69.5;
    public static final double ShoulderMidAngleDegrees = -13;
    public static final double ShoulderHighAngleDegrees = -1;
    public static final double ShoulderSoftUpperLimitDegrees = 0;
    public static final double ShoulderNeutralAngle = -40;

    public static final double WristSoftUpperLimitDegrees = 90;
    public static final double WristStowAngleDegrees = 74;
    public static final double WristHomeAngleDegrees = 62;
    public static final double WristFloorAngleDegrees = -4.5;
    public static final double WristScoringAngleDegrees = 0;
    public static final double WristSoftLowerLimitDegrees = -90;

    public static final double WristToArmSoftUpperLimitDegrees = 146.0;

    // Intake parameters
    public static final double IntakeMotorSpeed = 0.5;

    // Drivetrain low & high 'gear' speeds
    public static final double TurboSpeedMultiple = 0.90;
    public static final double HighSpeedMultiple = 0.65;
    public static final double LowSpeedMultiple = 0.30;
    
    // Drivetrain slew rate limits (input/sec)
    public static final double PanRateLimit = 8.0;
    public static final double TurnRateLimit = 8.0;

    // Arm rate limits
    public static final double ShoulderRateLimitDegPerSec = 40.0;
    public static final double ShoulderAccelLimitDegPerSecSquared = 40.0;
    public static final double WristRateLimitDegPerSec = 70.0;
    public static final double WristAccelLimitDegPerSecSquared = 100.0;

    public static final double ArmMotorMovementThresholdEncPer100ms = 300;
    public static final double ArmPidDemandThresholdPercent = 0.05;
    
    // Autonomous (PathPlanner) path following PID gains
    public static final double AutoDriveP = 0.75;
    public static final double AutoDriveI = 0.1;
    public static final double AutoDriveD = 0;
    public static final double AutoRotP = 0.5;
    public static final double AutoRotI = 0;
    public static final double AutoRotD = 0;

    // Balance PID gains
    public static final double BalanceP = 0.066;
    public static final double BalanceI = 0.001;
    public static final double BalanceD = 0;

    public static final int TalonEncoderTicksPerRev = 2048;
    public static final double ArmGearRatio = 140.0;
    public static final double ArmTalonEncoderCoeff = (360.0*100)/(TalonEncoderTicksPerRev*ArmGearRatio);
}
