// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveMod;

import java.net.Authenticator.RequestorType;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */



public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final SwerveDrivetrain drivetrain = SwerveMod.train;
  private final SwerveRequest.FieldCentric drivingRequest = new SwerveRequest.FieldCentric().withDeadband(SwerveMod.MaxSpeed
  * 0.05)
  .withRotationalDeadband(SwerveMod.MaxAngularSpeed * 0.05)
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
  .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  private final XboxController driverController = new XboxController(0);


  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

 
  private void configureBindings() {

    drivetrain.setControl(
        drivingRequest.withVelocityX(driverController.getLeftY())
            .withVelocityY(driverController.getLeftX())
            .withRotationalRate(driverController.getRightX()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
