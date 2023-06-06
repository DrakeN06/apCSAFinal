// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmMotorSubsystem;

public class PIDArm extends CommandBase {
  /** Creates a new PIDArm. */
  private PIDController pidController;
  private ArmMotorSubsystem armMotor;
  public PIDArm(ArmMotorSubsystem aMS, double setpoint) {
    armMotor = aMS;
    pidController = new PIDController(setpoint, setpoint, setpoint);
    pidController.setSetpoint(setpoint);
    addRequirements(armMotor);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Arm PID started.");
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double value = pidController.calculate(armMotor.getEncoderLocations());
    armMotor.setArmMotor(value);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Arm PID ended.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
