package frc.robot.codebases;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.DriveSubsystem;

public class PositionChecker {
    
    private final DriveSubsystem m_driveSubsystem;

    private double[] m_desiredPose = new double[4];
    private double m_desiredX;
    private double m_desiredY;
    private double m_desiredTheta;
    private double m_desiredSpeed; //for the shooter

    private Pose2d m_botPose;

    private double m_poseMargin;
    private double m_thetaMargin;
   
    public PositionChecker(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;

    }

    public void config(double[] desiredPosition, double poseMargin, double thetaMargin) {

        m_desiredX = desiredPosition[1];
        m_desiredY = desiredPosition[2];
        m_desiredTheta = desiredPosition[3];
        m_desiredSpeed = desiredPosition[4]; //for the shooter

        m_poseMargin = poseMargin;
        m_thetaMargin = thetaMargin;

    }

    

    
}
