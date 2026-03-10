// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.codebases.controllers.REVController;
import frc.robot.codebases.controllers.XboxControllerWrapper;
import frc.robot.limelightlib.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.codebases.controllers.Controller;
import frc.robot.codebases.controllers.PS4ControllerWrapper;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.climberSubsystem;
import frc.robot.subsystems.conveyerSubsystem;
import frc.robot.subsystems.intakeSubsystem;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final LimelightSubsystem m_limelight = new LimelightSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final conveyerSubsystem m_conveyer = new conveyerSubsystem();
  private final climberSubsystem m_climber = new climberSubsystem();
  private final intakeSubsystem m_intake = new intakeSubsystem();

  // The driver's controller
  private final REVController m_REVControllerDriver = new REVController(0);
  private final PS4ControllerWrapper m_PS4ControllerDriver = new PS4ControllerWrapper(0);
  private final XboxControllerWrapper m_XboxControllerDriver = new XboxControllerWrapper(0);
  private final REVController m_REVControllerTech = new REVController(1);
  private final PS4ControllerWrapper m_PS4ControllerTech = new PS4ControllerWrapper(1);
  private final XboxControllerWrapper m_XboxControllerTech = new XboxControllerWrapper(1);
  private final SendableChooser<GenericHID> m_controllerChooserDriver = new SendableChooser<>();
  private final SendableChooser<GenericHID> m_controllerChooserTech = new SendableChooser<>();
  // rotation controller
  private final PIDController turnController = new PIDController(0.65, 0, 0.01); 
  // distance controller
  private final PIDController driveController = new PIDController(0.4, 0, 0.02);
  // LR controller
  private final PIDController lrController = new PIDController(0.6, 0, 0.02);

  private final SlewRateLimiter rateLimiter = new SlewRateLimiter(1.5);

  private boolean fieldOriented = true;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //Configure the controller chooser
    m_controllerChooserDriver.setDefaultOption("PS4", m_PS4ControllerDriver); //options
    m_controllerChooserDriver.addOption("REV", m_REVControllerDriver);
    m_controllerChooserDriver.addOption("Xbox", m_XboxControllerDriver);
    SmartDashboard.putData("Driver (port 0 !!!)", m_controllerChooserDriver); //put it on the dashboard
    m_controllerChooserTech.setDefaultOption("PS4", m_PS4ControllerTech); //options
    m_controllerChooserTech.addOption("REV", m_REVControllerTech);
    m_controllerChooserTech.addOption("Xbox", m_XboxControllerTech);
    SmartDashboard.putData("Tech (port 1 !!!)", m_controllerChooserTech); //put it on the dashboard
    SmartDashboard.putBoolean("Field Oriented", true);

    

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
    new RunCommand(() -> {

        boolean goAttackMode =
            m_limelight.hasTarget()
            && (LimelightHelpers.getFiducialID("limelight") == 10
                || LimelightHelpers.getFiducialID("limelight") == 26)
            && getControllerDriver().getSquareButton();

        if (goAttackMode) {
            double turnSpeed = turnController.calculate(m_limelight.getTagYaw(), 0);
            double driveSpeed = driveController.calculate(m_limelight.getZ3d(), 2.25);
            double lrSpeed = lrController.calculate(m_limelight.getX3d(), 0);
            driveSpeed = rateLimiter.calculate(driveSpeed);
            //lrSpeed = rateLimiter.calculate(lrSpeed);
            //turnSpeed = rateLimiter.calculate(turnSpeed);

            m_robotDrive.drive(
                -driveSpeed,
                lrSpeed,
                turnSpeed,
                false
            );

            //System.out.println("TARGETTING!!!");
        } else {
           fieldOriented = SmartDashboard.getBoolean("Field Oriented", true);
            m_robotDrive.drive(
                -MathUtil.applyDeadband(getControllerDriver().getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(getControllerDriver().getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(getControllerDriver().getRightX(), OIConstants.kDriveDeadband),
                fieldOriented
            );

            //System.out.println("teleop...");
          }

      }, m_robotDrive)
    );

    m_conveyer.setDefaultCommand(
      new RunCommand(
        () -> m_conveyer.setConveyerSpeed(getControllerTech().getL2Axis()),
        m_conveyer));

    //Smart Dashboard Buttons
    SmartDashboard.putData("Reset Gyro", new InstantCommand(() -> m_robotDrive.zeroHeading()));
    SmartDashboard.putNumber("Shooter Speed (m/s)", 5); //default speed for the shooter, can be adjusted on the dashboard
   
  }

  /**
   * Returns the currently selected controller.
   */
  public GenericHID getControllerHIDDriver() {
    return m_controllerChooserDriver.getSelected();
  }

  public Controller getControllerDriver() {
    return (Controller) m_controllerChooserDriver.getSelected();
  }

  public GenericHID getControllerHIDTech() {
    return m_controllerChooserTech.getSelected();
  }

  public Controller getControllerTech() {
    return (Controller) m_controllerChooserTech.getSelected();
  }

  
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(getControllerHIDDriver(), PS4Controller.Button.kSquare.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    /* 
    new JoystickButton(getControllerHIDTech(), PS4Controller.Button.kCircle.value)
        .whileTrue(new RunCommand(
            () -> m_shooter.setShooterSpeedPID(SmartDashboard.getNumber("Shooter Speed (m/s)", 5)),
            m_shooter));
    */
    new JoystickButton(getControllerHIDDriver(), PS4Controller.Button.kTriangle.value)
      .whileTrue(new StartEndCommand(
        () ->  m_climber.setclimberSpeed(0.5), 
        () -> m_climber.stopClimber(), 
        m_climber));
    new JoystickButton(getControllerHIDDriver(), PS4Controller.Button.kCross.value)
      .whileTrue(new StartEndCommand(
        () ->  m_climber.setclimberSpeed(-0.5), 
        () -> m_climber.stopClimber(),  
        m_climber));
    new JoystickButton(getControllerHIDTech(), PS4Controller.Button.kSquare.value)
      .whileTrue(new ParallelCommandGroup(
        new RunCommand(() -> m_shooter.setShooterSpeed(1.0), m_shooter),
        new SequentialCommandGroup(
          new WaitCommand(1.5),
          new RunCommand(() -> m_conveyer.setConveyerSpeed(0.85), m_conveyer)
        )))
        .whileFalse(new RunCommand(() -> m_shooter.stopShooter(), m_shooter));
    new JoystickButton(getControllerHIDTech(), PS4Controller.Button.kCircle.value)
        .whileTrue(new RunCommand(
            () -> m_intake.setFolderSpeed(1),
            m_intake))
        .whileFalse(new RunCommand(
            () -> m_intake.setFolderSpeed(0),
            m_intake));
    new JoystickButton(getControllerHIDTech(), PS4Controller.Button.kCross.value)
      .whileTrue(new RunCommand(() -> m_intake.setIntakeSpeed(0.5), m_intake))
      .whileFalse(new RunCommand(() -> m_intake.stopIntake(), m_intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
       List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);
      Trajectory basicAutoTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0,new Rotation2d(0)),
        List.of(),
        new Pose2d(1,-0.15,new Rotation2d(3)),
        config
      );

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        basicAutoTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(basicAutoTrajectory.getInitialPose());
    //return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    return new SequentialCommandGroup(
      new ParallelRaceGroup(
        new RunCommand(() -> m_shooter.stopShooter(), m_shooter),
        new RunCommand(() -> m_conveyer.setConveyerSpeed(0), m_conveyer),
        swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false))
      ),
      new ParallelDeadlineGroup(
        new RunCommand(() -> m_shooter.setShooterSpeed(1.0), m_shooter),
        new SequentialCommandGroup(
          new WaitCommand(1.5),
          new RunCommand(() -> m_conveyer.setConveyerSpeed(0.85), m_conveyer)
        ),
        new WaitCommand(4)
      ),
      new ParallelCommandGroup(
        new RunCommand(() -> m_shooter.stopShooter(), m_shooter),
        new RunCommand(() -> m_conveyer.setConveyerSpeed(0), m_conveyer)
      )
    );
  }
}
