// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.codebases.controllers.Controller;
import frc.robot.codebases.controllers.PS4ControllerWrapper;
import frc.robot.codebases.controllers.REVController;
import frc.robot.codebases.controllers.XboxControllerWrapper;
import frc.robot.commands.AssistedShoot;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.Climb;
import frc.robot.commands.Fold;
import frc.robot.commands.Intake;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FolderSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

//SUBSYSTEMS
  public final DriveSubsystem m_driver = new DriveSubsystem();
  public final LimelightSubsystem m_limelight = new LimelightSubsystem();
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public final ConveyerSubsystem m_conveyer = new ConveyerSubsystem();
  public final ClimberSubsystem m_climber = new ClimberSubsystem();
  public final IntakeSubsystem m_intake = new IntakeSubsystem();
  public final FolderSubsystem m_folder = new FolderSubsystem();

//CONTROLLER
  //Driving Controller
  private final REVController m_REVControllerDriver = new REVController(0);
  private final PS4ControllerWrapper m_PS4ControllerDriver = new PS4ControllerWrapper(0);
  private final XboxControllerWrapper m_XboxControllerDriver = new XboxControllerWrapper(0);
  private final SendableChooser<GenericHID> m_controllerChooserDriver = new SendableChooser<>();

  //Tech Controller
  private final REVController m_REVControllerTech = new REVController(1);
  private final PS4ControllerWrapper m_PS4ControllerTech = new PS4ControllerWrapper(1);
  private final XboxControllerWrapper m_XboxControllerTech = new XboxControllerWrapper(1);
  private final SendableChooser<GenericHID> m_controllerChooserTech = new SendableChooser<>();

//AUTO

  private final SendableChooser<Command> autoChooser;

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

  // Intialize choosers for the controllers 
    //driver controller
    m_controllerChooserDriver.setDefaultOption("PS4", m_PS4ControllerDriver); //options
    m_controllerChooserDriver.addOption("REV", m_REVControllerDriver);
    m_controllerChooserDriver.addOption("Xbox", m_XboxControllerDriver);
    SmartDashboard.putData("Driver (port 0 !!!)", m_controllerChooserDriver); //put it on the dashboard

    //tech controller
    m_controllerChooserTech.setDefaultOption("PS4", m_PS4ControllerTech); //options
    m_controllerChooserTech.addOption("REV", m_REVControllerTech);
    m_controllerChooserTech.addOption("Xbox", m_XboxControllerTech);
    SmartDashboard.putData("Tech (port 1 !!!)", m_controllerChooserTech); //put it on the dashboard
    SmartDashboard.putBoolean("Field Oriented", true);

    //AUTO
    configureAutoCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    
    //NamedCommands.registerCommand("Folder Down", new Fold(m_folder, false));
    //NamedCommands.registerCommand("Intake", new Intake(m_intake, true));

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_driver.setDefaultCommand( 
      new RunCommand(  
        () -> m_driver.drive(
                -MathUtil.applyDeadband(getControllerDriver().getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(getControllerDriver().getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_REVControllerDriver.getRawAxis(4), OIConstants.kDriveDeadband), //rightX
                DriveConstants.kFeildOreiented)
            , m_driver) );
  


    

    //Smart Dashboard Buttons
    SmartDashboard.putData("Reset Gyro", new InstantCommand(() -> m_driver.zeroHeading()));
    SmartDashboard.putNumber("Shooter Speed", 1); //default speed for the shooter, can be adjusted on the dashboard
   
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

  
  //BUTONS

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
    
    //X Wheel Formation
    new JoystickButton(getControllerHIDDriver(), 1) //Cross
        .whileTrue(new RunCommand(
            () -> m_driver.setX(),
            m_driver));

    /* 
    new JoystickButton(getControllerHIDTech(), PS4Controller.Button.kCircle.value)
        .whileTrue(new RunCommand(
            () -> m_shooter.setShooterSpeedPID(SmartDashboard.getNumber("Shooter Speed (m/s)", 5)),
            m_shooter));
    */

    //Climb Up
    new JoystickButton(getControllerHIDDriver(), 4) //Triangle
      .whileTrue(new Climb(m_climber, true));

    //Climb Down
    new JoystickButton(getControllerHIDDriver(),2) //Circles
      .whileTrue(new Climb(m_climber, false));
    
    //Shoot
    new JoystickButton(getControllerHIDTech(), PS4Controller.Button.kSquare.value)
      .whileTrue(new Shoot(m_shooter, m_conveyer, SmartDashboard.getNumber("Shooter Speed", ShooterConstants.kShooterSpeedDefault)));

    //Put Folder Down
    new JoystickButton(getControllerHIDTech(), PS4Controller.Button.kCircle.value)
        .whileTrue(new Fold(m_folder, false));

    //Put Folder UP
    new JoystickButton(getControllerHIDTech(), PS4Controller.Button.kTriangle.value)
        .whileTrue(new Fold(m_folder, true));

    //Intake
    new JoystickButton(getControllerHIDTech(), PS4Controller.Button.kCross.value)
      .whileTrue(new Intake(m_intake, true));

    

    //Specail: run right shooter
    new JoystickButton(getControllerHIDDriver(), 3) //square
      .whileTrue(new AssistedShoot(m_driver, m_limelight));

  }

  public void configureAutoCommands() {

    NamedCommands.registerCommand("Shoot", new AutoShoot(m_shooter, m_conveyer, ShooterConstants.kShooterSpeedDefault));

  }



  public Command getPPAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   *
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
      
      //Our Trajectory  
      Trajectory basicAutoTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0,new Rotation2d(0)),
        List.of(),
        new Pose2d(1,1,new Rotation2d(3)),
        config
      );

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        basicAutoTrajectory,
        m_driver::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_driver::setModuleStates,
        m_driver);

    // Reset odometry to the starting pose of the trajectory.
    m_driver.resetOdometry(basicAutoTrajectory.getInitialPose());
    //return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));

    //returns the actual commands to be ran
    return new SequentialCommandGroup(
      new ParallelRaceGroup(
        new RunCommand(() -> m_shooter.stopShooter(), m_shooter),
        new RunCommand(() -> m_conveyer.setConveyerSpeed(0), m_conveyer),
        swerveControllerCommand.andThen(() -> m_driver.drive(0, 0, 0, false))
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
  }*/
}
