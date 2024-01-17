// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



/*
    Shooter/intake:

    On the Right side 
 * I need a shooter that shoots outward when I press a button,
 * I want one fourth powers for all
 * BUTTT I want another button to retract when I press a button
 * 
 * 2 victor motorcontrollers
 * 
 * 
 */

public class shooter extends SubsystemBase {
  /** Creates a new shooter. */
    VictorSPX Shooter = new VictorSPX(Constants.ShooterA);
    VictorSPX intake = new VictorSPX(Constants.ShooterB);


  public shooter() {
    Shooter.configFactoryDefault();
    intake.configFactoryDefault();

    Shooter.setNeutralMode(NeutralMode.Brake);
    intake.setNeutralMode(NeutralMode.Coast);

    intake.follow(Shooter);
  }


public void shoot(boolean A,boolean B){
    //A intakes
    if(A){
        Shooter.set(ControlMode.PercentOutput, 0.25);
    }
    //B shoots 
    else if(B){
        Shooter.set(ControlMode.PercentOutput, -0.25);
    }
    else{
        Shooter.set(ControlMode.PercentOutput, 0);
    }


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
