package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;

import edu.wpi.first.math.util.Units;

public class SwerveMod {
    private static final String CANbusName = "drivetrain";
/*
 * Might need to Change the Gains, Trackwidth, Wheelbase, IDs, and offset
 */
    private static final double DriveMotorGR = 6.75;
    private static final double SteerMotorGR = 15.43;

    private static final Slot0Configs SteerGain = new Slot0Configs()
    .withKP(10)
    .withKI(0)
    .withKD(0.2)
    .withKA(0)
    .withKV(1.5)
    .withKS(0);

    private static final Slot0Configs DriveGain = new Slot0Configs()
    .withKP(3)
    .withKI(0)
    .withKD(0)
    .withKA(0)
    .withKS(0)
    .withKV(0);

    private static final double WheelRad = 2;

    public static final double MaxSpeed = (6380/6.75* Math.PI * 4/12 / 60);
    public static final double MaxAngularSpeed = 1.5*Math.PI;

    private static final ClosedLoopOutputType SteerLoopOutput = ClosedLoopOutputType.Voltage;

    private static final ClosedLoopOutputType DriveLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;

   


    public static final SwerveDrivetrainConstants DriveConstant = new SwerveDrivetrainConstants()
    .withPigeon2Id(0).withCANbusName(CANbusName);

// SwerveDriveConstants factory = Settings

public static final SwerveModuleConstantsFactory factorySettings = new SwerveModuleConstantsFactory()
.withDriveMotorGearRatio(DriveMotorGR)
.withSteerMotorGearRatio(SteerMotorGR)
.withWheelRadius(2)
.withDriveMotorGains(DriveGain)
.withSteerMotorGains(SteerGain)
.withDriveMotorClosedLoopOutput(DriveLoopOutput)
.withSteerMotorClosedLoopOutput(SteerLoopOutput)
.withSpeedAt12VoltsMps(MaxSpeed)
.withSteerMotorInverted(false);     

private static final double trackWidth = 20;
private static final double wheelbase = 20;

private static final int FRightDriveID = 10;
private static final int FRightSteerID = 20;
private static final int FRightSteerEncoderID = 30;
private static final double FRightSteerOffset = 40;

private static final int FLeftDriveID = 0;
private static final int FLeftSteerID = 0;
private static final int FLeftSteerEncoder = 0;
private static final double FLeftSteerOffset = 0;

private static final int BRightDriveID = 0;
private static final int BRightSteerId = 0;
private static final int BRightSteerEncoder = 0;
private static final double BRightSteerOffset = 0;

private static final int BLeftDriveID = 0;
private static final int BLeftSteerID = 0;
private static final int BLeftSteerEncoderID = 0;
private static final double BLeftSteerOffset = 0;




private static final SwerveModuleConstants FrontRight = factorySettings.createModuleConstants(
    FRightSteerID
    ,FRightDriveID
    ,FRightSteerEncoderID
    ,FRightSteerOffset
    ,Units.inchesToMeters(trackWidth/2)
    ,Units.inchesToMeters(-wheelbase/2)
    ,false);
  
private static final SwerveModuleConstants FrontLeft = factorySettings.createModuleConstants(
    FLeftDriveID,
    FLeftSteerID,
    FLeftSteerEncoder,
    FLeftSteerOffset,
    Units.inchesToMeters(trackWidth/2),
    Units.inchesToMeters(wheelbase/2),
    false
);

private static final SwerveModuleConstants BackLeft = factorySettings.createModuleConstants(
    BLeftDriveID,
    BLeftSteerID,
    BLeftSteerEncoderID,
    BLeftSteerOffset,
    Units.inchesToMeters(-trackWidth/2),
    Units.inchesToMeters(wheelbase/2),
    false
);

private static final SwerveModuleConstants BackRight = factorySettings.createModuleConstants(
    BRightSteerId
    ,BRightDriveID
    ,BRightSteerEncoder
    ,BRightSteerOffset
    ,Units.inchesToMeters(-trackWidth/2)
    ,Units.inchesToMeters(-wheelbase/2)
    ,false);

public static final SwerveDrivetrain train = new SwerveDrivetrain(DriveConstant
, FrontLeft,
FrontRight,
BackLeft,
BackRight);

}
