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
  
  private final SparkMax m_shooterSpark1;
  private final SparkMax m_shooterSpark2;

  private final RelativeEncoder m_shooterEncoder1;
  private final RelativeEncoder m_shooterEncoder2;

  private final SparkClosedLoopController m_shooterClosedLoopController1;
  private final SparkClosedLoopController m_shooterClosedLoopController2;
 

  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    

    m_shooterSpark1 = new SparkMax(9, SparkMax.MotorType.kBrushless); //is it brushless or brushed? check the wiring and the motor
    m_shooterSpark2 = new SparkMax(14, SparkMax.MotorType.kBrushless);
    m_shooterSpark1.configure(Configs.ShooterSparkMax.ShooterConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_shooterSpark2.configure(Configs.ShooterSparkMax.ShooterConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    m_shooterEncoder1 = m_shooterSpark1.getEncoder();
    m_shooterEncoder2 = m_shooterSpark2.getEncoder();

    m_shooterClosedLoopController1 = m_shooterSpark1.getClosedLoopController();
    m_shooterClosedLoopController2 = m_shooterSpark2.getClosedLoopController();
      

  }

  @Override
  public void periodic() {

   
  }

  public void setShooterSpeed(double speed) { //out of voltage -1 to 1
    m_shooterSpark1.set(speed);
    m_shooterSpark2.set(speed);
  }

  public void setShooterLeft(double speed) { //out of voltage -1 to 1
    m_shooterSpark2.set(speed);
    System.out.println("Method");
  }

  public void setShooterRight(double speed) { //out of voltage -1 to 1
    m_shooterSpark1.set(speed);
  }

  /**
   * creates a PID to ramp up the speed of the shooter
   * 
   * @param speed is in m/s (idk what the limit is)
   */
  public void setShooterSpeedPID(double speed) { //speed in m/s
    m_shooterClosedLoopController1.setSetpoint(speed, SparkMax.ControlType.kVelocity);
    m_shooterClosedLoopController2.setSetpoint(speed, SparkMax.ControlType.kVelocity);
  }

  public void stopShooter() {
    m_shooterSpark1.stopMotor();
    m_shooterSpark2.stopMotor();
  }

}

 
