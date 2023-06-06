// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmMotorSubsystem extends SubsystemBase {
  /** Creates a new ArmMotorSubsystem. */
  private TalonFX armMotor;
  private final double  kDriveTick2Rotate = 1.0/2046.0;
  
  
  public ArmMotorSubsystem() {
    armMotor = new TalonFX(Constants.ARM_MOTOR);
  }

  public void setArmMotor(double speed){
    armMotor.set(ControlMode.PercentOutput, speed);

  }

  public void brakeMotor(){
    armMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void resetArmEncoders(){
    armMotor.setSelectedSensorPosition(0);
  }

  public double getEncoderLocations(){
    double value = armMotor.getSelectedSensorPosition()*kDriveTick2Rotate;
    System.out.println("rotate: " + value);
    return value;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
