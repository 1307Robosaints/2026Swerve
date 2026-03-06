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





public class intakeSubsystem extends SubsystemBase {
  
  private final SparkMax m_folderSpark;
  private final SparkMax m_intakeSpark;


  private final RelativeEncoder m_folderEncoder;
  private final SparkClosedLoopController m_folderClosedLoopController;
  private final ClosedLoopConfig m_folderClosedLoopConfig = new ClosedLoopConfig();

  
  /** Creates a new folderSubsystem. */
  public intakeSubsystem() {
    
    //folder:

    m_folderClosedLoopConfig
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(0.15, 0, 0.005)
      .outputRange(0, 1);

    m_folderSpark = new SparkMax(12, SparkMax.MotorType.kBrushless); //is it brushless or brushed? check the wiring and the motor
    m_folderSpark.configure(Configs.folderSparkMax.folderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_folderEncoder = m_folderSpark.getEncoder();

    m_folderClosedLoopController = m_folderSpark.getClosedLoopController();
    
      //intake:

    m_intakeSpark = new SparkMax(13, SparkMax.MotorType.kBrushless);
    m_intakeSpark.configure(Configs.intakeSparkMax.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {

    
   
  }

  //folder:

  public void setFolderSpeed(double speed) {
    m_folderSpark.set(speed);
  }

  public void stopFolder() {
    m_folderSpark.set(0);
  }

  public void setFolderPosition(double position) { //speed in m/s
    m_folderClosedLoopController.setSetpoint(position, SparkMax.ControlType.kPosition);
  }

  //intake:

  public void stopIntake() {
    m_intakeSpark.set(0);
  }

  public void setIntakeSpeed(double speed) {
    m_intakeSpark.set(speed);
  }

}

 
