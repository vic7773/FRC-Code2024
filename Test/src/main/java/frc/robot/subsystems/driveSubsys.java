// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class driveSubsys extends SubsystemBase {
  TalonSRX FR = new TalonSRX(Constants.FrontR);
  TalonSRX FL = new TalonSRX(Constants.FrontL);
  TalonSRX BR = new TalonSRX(Constants.BackR);
  TalonSRX BL = new TalonSRX(Constants.BackL);
  /** Creates a new driveSubsys. */
  public driveSubsys() {
    FR.configFactoryDefault();
    FL.configFactoryDefault();
    BR.configFactoryDefault();
    BL.configFactoryDefault();

    FR.setNeutralMode(NeutralMode.Brake);
    BR.setNeutralMode(NeutralMode.Brake);
    FL.setNeutralMode(NeutralMode.Brake);
    BL.setNeutralMode(NeutralMode.Brake);

    BR.follow(FR);
    BL.follow(FL);

    FR.setInverted(InvertType.InvertMotorOutput);
    BR.setInverted(InvertType.FollowMaster);

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
 public void motorpower(double power, double turn){
    FR.set(ControlMode.PercentOutput, power-turn);
    FL.set(ControlMode.PercentOutput, power+turn);
 }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
