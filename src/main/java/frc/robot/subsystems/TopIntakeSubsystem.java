// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TopIntakeSubsystem extends SubsystemBase {
  /** Creates a new TopIntakeSubsystem. */
  private WPI_TalonFX topMotor;
  public TopIntakeSubsystem() {
    topMotor = new WPI_TalonFX(Constants.TOP_MOTOR);
  }

  public void setTopMotor(double speed){
    topMotor.set(ControlMode.PercentOutput, speed);
  }

  public void brakeTopIntake(){
    topMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
