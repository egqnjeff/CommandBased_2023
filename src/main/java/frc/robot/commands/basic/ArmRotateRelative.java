// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.constants.ArmConstants;
import frc.robot.RobotShared;
import frc.robot.subsystems.ArmProfiledPIDSubsystem;

public class ArmRotateRelative extends CommandBase {

  private ArmProfiledPIDSubsystem m_armProfiled;
  private RobotShared m_robotShared;
  private double m_relativeDegrees;
  private double m_target;
  private double m_speed;

  /** Creates a new ArmRotateRelative. */
  public ArmRotateRelative(double relativeDegrees) {
    m_robotShared = RobotShared.getInstance();
    m_armProfiled = m_robotShared.getArmProfiledPIDSubsystem();
    m_relativeDegrees = relativeDegrees;
    addRequirements(m_armProfiled);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_target = m_armProfiled.getArmRotationDegrees() + m_relativeDegrees;
    m_speed = Math.copySign(ArmConstants.kDunkRotationSpeedMagnitude, m_relativeDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armProfiled.manualArmRotate(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armProfiled.manualDone();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentDegrees = m_armProfiled.getArmRotationDegrees();
    if(m_speed > 0) {
      return currentDegrees >= m_target;
    }
    return currentDegrees <= m_target;
  }
}
