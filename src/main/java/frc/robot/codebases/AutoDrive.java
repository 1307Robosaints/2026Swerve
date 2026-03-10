package frc.robot.codebases;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;

public class AutoDrive {

    private final DriveSubsystem m_subsystem;
    private ProfiledPIDController m_thetaController;
    private Trajectory m_trajectory;
    private SwerveControllerCommand swerveControllerCommand;

    public AutoDrive(DriveSubsystem subsystem ) {
        m_subsystem = subsystem;
        
    }

    public void configure() {
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory m_trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

        m_thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void run() {

        swerveControllerCommand = new SwerveControllerCommand(
            m_trajectory,
            m_subsystem::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            m_thetaController,
            m_subsystem::setModuleStates,
            m_subsystem);
    }

    public SwerveControllerCommand getSwerveController() {

        return swerveControllerCommand;
    }

    public boolean isFinished(){
        boolean finished;

        finished = swerveControllerCommand.isFinished();

        return finished;
    }








}
