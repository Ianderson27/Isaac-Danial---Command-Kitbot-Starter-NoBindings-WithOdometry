package frc.robot.commands;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;



public class trajectorys extends Command {
  private final DriveSubsystem m_drivetrain;
  private final DifferentialDriveKinematics m_kinematics;
  private final RamseteController m_ramseteController;
  private final Timer m_timer = new Timer();
  
  // Static trajectory generated internally
  private final Trajectory m_trajectory;
  // Drivetrain parameters & trajectory limits
  private static final double kTrackWidthMeters = 0.55; // Distance between left and right wheels
  private static final double kMaxVelocityMetersPerSec = 2.5;
  private static final double kMaxAccelerationMetersPerSecSq = 1.0;




  public trajectorys(DriveSubsystem drivetrain) {

    m_drivetrain = drivetrain;
    m_kinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
    m_ramseteController = new RamseteController();
    // 1. Configure speed & acceleration constraints
    TrajectoryConfig config = new TrajectoryConfig(
        kMaxVelocityMetersPerSec, 
        kMaxAccelerationMetersPerSecSq
    ).setKinematics(m_kinematics);
    // Pose2d startPose = new Pose2d(0.5, 4.1, Rotation2d.fromDegrees(0));

    // //2. Generate static trajectory points directly in the constructor
    // m_trajectory = TrajectoryGenerator.generateTrajectory(
    //     startPose,                // Start pose (x, y, heading)
    //     List.of(new Translation2d(1, 1),
    //             new Translation2d(2,2)                                ),               // Interior waypoints (x, y)
    //     new Pose2d(3, 3, new Rotation2d(0.7853981634)),              // End pose (x, y, heading)
    //     config
    // );
// 1. Define starting pose (centered on alliance wall facing downfield)
Pose2d startPose = new Pose2d(0.5, 4.1, Rotation2d.fromDegrees(0));

// 2. Define interior waypoints around the hub
List<Translation2d> interiorWaypoints = List.of(
    new Translation2d(2.0, 1.8), // Veer right toward the bump
    new Translation2d(4.5, 1.8), // Travel under the bump past the hub
    new Translation2d(6.0, 4.1), // Round behind the hub in the center
    new Translation2d(4.5, 6.4), // Return along the left side of the hub
    new Translation2d(2.0, 6.4)  // Head back toward alliance wall
);

// 3. Define ending pose at roughly the starting position
// Note: Facing 180 degrees because the robot arrives traveling back (-X direction)
Pose2d endPose = new Pose2d(0.5, 4.1, Rotation2d.fromDegrees(180));

// 4. Generate the full loop trajectory
m_trajectory = TrajectoryGenerator.generateTrajectory(
    startPose,
    interiorWaypoints,
    endPose,
    config
);
    //     m_trajectory = TrajectoryGenerator.generateTrajectory(
    //     new Pose2d(0, 0, new Rotation2d(0)),                // Start pose (x, y, heading)
    //     List.of(new Translation2d(1, 0),
    //             new Translation2d(2,0)                                ),               // Interior waypoints (x, y)
    //     new Pose2d(3, 0, new Rotation2d(0)),              // End pose (x, y, heading)
    //     config
    // );
    addRequirements(m_drivetrain);
  }
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    // Reset field position tracking to match the trajectory start pose [1]
    
    Pose2d startPose = new Pose2d(0.5, 4.1, Rotation2d.fromDegrees(0));

    m_drivetrain.resetPose(startPose);  
}

  @Override
  public void execute() {
    double elapsed = m_timer.get();
    // Sample target position along the trajectory at the current timestamp
    Trajectory.State goal = m_trajectory.sample(elapsed);
    // Calculate required chassis speeds to follow target pose [1]
    ChassisSpeeds targetSpeeds = m_ramseteController.calculate(m_drivetrain.getPose(), goal);
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(targetSpeeds);
    // Command drivetrain wheel speeds
    m_drivetrain.driveWheelSpeeds(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);

    SmartDashboard.putNumber(
    "Trajectory Start Heading (deg)",
    m_trajectory.getInitialPose().getRotation().getDegrees()
    );

    SmartDashboard.putNumber(
        "Trajectory End Heading (deg)",
        m_trajectory.sample(m_trajectory.getTotalTimeSeconds())
            .poseMeters.getRotation().getDegrees()
    );
    SmartDashboard.putNumber(
        "goal heading (deg)",
        m_trajectory.sample(elapsed).poseMeters.getRotation().getDegrees()
    );

    SmartDashboard.putNumber(
        "Trajectory Current Heading (deg)",
        m_drivetrain.getPose().getRotation().getDegrees()
    );
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_drivetrain.arcadeDriveCommand(()->0, ()->0); // Stop drivetrain motors
  }
}