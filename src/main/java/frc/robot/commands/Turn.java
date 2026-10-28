// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;



/** An example command that uses an example subsystem. */
public class Turn extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

    private final double m_kP = 0.01;
    private final double m_maxTurnSpeed = 1;
    private final double m_toleranceDegrees = 2.0;
    private final double m_minTurnSpeed = 0.1;


  double heading;
  
  private DriveSubsystem m_drive;

  public Turn(DriveSubsystem Drive, double Heading) {
    m_drive = Drive;
    heading = Heading;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentHeading = m_drive.getPose().getRotation().getDegrees();

    double headingError = MathUtil.inputModulus((heading-currentHeading),-180,180);

    double distance =  MathUtil.clamp(headingError * m_kP,-m_maxTurnSpeed, m_maxTurnSpeed);

    if (distance > 0 && distance < m_minTurnSpeed){
        distance = m_minTurnSpeed;
    }
    else if(distance < 0 && distance > - m_minTurnSpeed){ 
        distance = (- m_minTurnSpeed);
    }

    final double distanceFinal = distance;


    SmartDashboard.putNumber("Current speede", distance);
    SmartDashboard.putNumber("Current heading", currentHeading);
    SmartDashboard.putNumber("target heading", heading);

    m_drive.m_differentialDrive.arcadeDrive( 0,  distanceFinal);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.m_differentialDrive.arcadeDrive( 0,  0
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentHeading = m_drive.getPose().getRotation().getDegrees();

    double headingError = Math.abs(MathUtil.inputModulus((currentHeading - heading),-180,180));

    if(headingError < m_toleranceDegrees){
        return true;
    }
    else{
        return false;
    }


    }
  }

