// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.ClosedLoopConfig;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.commands.Drive;





public class ShooterSubsystem extends SubsystemBase {
  
  private final SparkMax m_shooterSpark;


  private final RelativeEncoder m_shooterEncoder;
  private final SparkClosedLoopController m_shooterClosedLoopController;
  private final ClosedLoopConfig m_shooterClosedLoopConfig = new ClosedLoopConfig();
;

  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    


    m_shooterClosedLoopConfig
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(0.04, 0, 0)
      .velocityFF(.05) //I need to redo this with diffrernt method later
      .outputRange(0, 1);

    m_shooterSpark = new SparkMax(9, SparkMax.MotorType.kBrushless); //is it brushless or brushed? check the wiring and the motor
    m_shooterSpark.configure(Configs.ShooterSparkMax.ShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_shooterEncoder = m_shooterSpark.getEncoder();

    m_shooterClosedLoopController = m_shooterSpark.getClosedLoopController();
    
      

  }

  @Override
  public void periodic() {

    
   
  }

  public void setShooterSpeed(double speed) { //out of voltage -1 to 1
    m_shooterSpark.set(speed);
  }

  public void setShooterSpeedPID(double speed) { //speed in m/s
    m_shooterClosedLoopController.setSetpoint(speed, SparkMax.ControlType.kVelocity);
  }

  public void stopShooter() {
    m_shooterSpark.stopMotor();
  }

}

 
