// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  private WPI_TalonFX leftMotor1;
  private WPI_TalonFX leftMotor2;
  private WPI_TalonFX rightMotor1;
  private WPI_TalonFX rightMotor2;

  MotorControllerGroup leftGroup;
  MotorControllerGroup rightGroup;

  DifferentialDrive myDrive;
  private final double tick2Feet = 1.0/128*Math.PI/12;




  public DrivetrainSubsystem() {
    leftMotor1 = new WPI_TalonFX(Constants.LEFT_FRONT_MOTOR);
    leftMotor2 = new WPI_TalonFX(Constants.LEFT_REAR_MOTOR);
    rightMotor1 = new WPI_TalonFX(Constants.RIGHT_FRONT_MOTOR);
    rightMotor2 = new WPI_TalonFX(Constants.RIGHT_REAR_MOTOR);

    leftGroup = new MotorControllerGroup(leftMotor1, leftMotor2);
    rightGroup = new MotorControllerGroup(rightMotor1, rightMotor2);

    myDrive = new DifferentialDrive(leftGroup, rightGroup);
  }

  //Write the methods: Get encoder feet, arcade drive, tank drive, curvature drive

  public double getDistance(){
    double leftValue = leftMotor1.getSelectedSensorPosition()*tick2Feet;
    double rightValue = rightMotor1.getSelectedSensorPosition()*tick2Feet;
    double distance = (leftValue + rightValue)/2.0;
    return distance;
  }

  public void arcadeDrive(double xSpeed, double zRotation){
    myDrive.arcadeDrive(xSpeed, zRotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    myDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void curvatureDrive(double xSpeed, double zRotation, boolean turn){
    myDrive.curvatureDrive(xSpeed, zRotation, turn);
  }

  public void stop(){
    //look
    myDrive.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
