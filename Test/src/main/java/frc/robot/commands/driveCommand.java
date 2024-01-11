// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.driveSubsys;

import java.util.function.DoubleSupplier;

import javax.print.attribute.standard.JobOriginatingUserName;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class driveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final driveSubsys drive;
  private final DoubleSupplier Fwd,Bck,turn;

  /**
   * Creates a new driveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public driveCommand(driveSubsys subsystem, DoubleSupplier Ftrig, DoubleSupplier Btrig, DoubleSupplier Jturn) {
    drive = subsystem;
    Fwd = Ftrig;
    Bck = Btrig;
    turn = Jturn;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.motorpower(Fwd.getAsDouble() - Bck.getAsDouble(), turn.getAsDouble());
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
