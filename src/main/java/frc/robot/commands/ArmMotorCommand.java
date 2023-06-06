// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmMotorSubsystem;

public class ArmMotorCommand extends CommandBase {
  /** Creates a new ArmMotorCommand. */
  private final ArmMotorSubsystem armM;
  private final Supplier<Double> speedFunction;
  public ArmMotorCommand(ArmMotorSubsystem arm, Supplier<Double> sF) {
    // Use addRequirements() here to declare subsystem dependencies.
    armM = arm;
    speedFunction = sF;
    addRequirements(armM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = speedFunction.get();
    armM.setArmMotor(speed);
    //armM is the subsystem. The command is telling the subsystem to do something.
    armM.brakeMotor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(" Arm Stopped.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
