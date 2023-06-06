// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TopIntakeSubsystem;

public class TopIntakeCommand extends CommandBase {
  /** Creates a new TopIntakeCommand. */
  private final TopIntakeSubsystem topM;
  private final Supplier<Double> speedFunction;
  
  public TopIntakeCommand(TopIntakeSubsystem tM, Supplier<Double> sF) {
    // Use addRequirements() here to declare subsystem dependencies.
    topM = tM;
    speedFunction = sF;
    addRequirements(topM);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = speedFunction.get();
    topM.setTopMotor(speed);
    topM.brakeTopIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Top Intake Stopped.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
