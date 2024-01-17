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

public class Subsys extends SubsystemBase {
  /** Creates a new Subsys. */
    TalonSRX FR = new TalonSRX(Constants.FR);
    TalonSRX FL = new TalonSRX(Constants.FL);
    TalonSRX BR = new TalonSRX(Constants.BR);
    TalonSRX BL = new TalonSRX(Constants.BL);

  public Subsys() {
    FR.configFactoryDefault();
    FL.configFactoryDefault();
    BR.configFactoryDefault();
    BL.configFactoryDefault();


    FR.setNeutralMode(NeutralMode.Coast);
    FL.setNeutralMode(NeutralMode.Coast);
    BR.setNeutralMode(NeutralMode.Coast);
    BL.setNeutralMode(NeutralMode.Coast);

    FR.setInverted(InvertType.InvertMotorOutput);
    BR.setInverted(InvertType.FollowMaster);

    BL.follow(FL);
    BR.follow(FR);

  }



public void turning(double power, double turn){
    FR.set(ControlMode.PercentOutput,power - turn);
    FL.set(ControlMode.PercentOutput,power + turn);
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
