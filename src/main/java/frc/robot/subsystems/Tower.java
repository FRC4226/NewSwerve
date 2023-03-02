// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;


import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax.ControlType;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class Tower extends SubsystemBase {

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private SparkMaxPIDController m_leftPID;
    private RelativeEncoder m_leftEncoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;




    public Tower() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
leftMotor = new CANSparkMax(30, MotorType.kBrushless);
 leftMotor.restoreFactoryDefaults();  
leftMotor.setInverted(false);
leftMotor.setIdleMode(IdleMode.kBrake);
m_leftPID = leftMotor.getPIDController();
m_leftEncoder = leftMotor.getEncoder();

rightMotor = new CANSparkMax(31, MotorType.kBrushless);
 rightMotor.restoreFactoryDefaults();  

rightMotor.setIdleMode(IdleMode.kBrake);
rightMotor.follow(leftMotor, true);

kP = 5e-5; 
kI = 1e-6;
kD = 0; 
kIz = 0; 
kFF = 0.00125; 
kMaxOutput = 1; 
kMinOutput = -1;
maxRPM = 5700;
maxVel = 700;
maxAcc = 700;

    m_leftPID.setP(kP);
    m_leftPID.setI(kI);
    m_leftPID.setD(kD);
    m_leftPID.setIZone(kIz);
    m_leftPID.setFF(kFF);
    m_leftPID.setOutputRange(kMinOutput, kMaxOutput);
  
     int smartMotionSlot = 0;
    m_leftPID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_leftPID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_leftPID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_leftPID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    @Override
    public void periodic() {

        double setPoint, processVariable;
        boolean mode = SmartDashboard.getBoolean("Mode", false);
        if(mode) {
          setPoint = SmartDashboard.getNumber("Set Velocity", 0);
          m_leftPID.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
          processVariable = m_leftEncoder.getVelocity();
        } else {
          setPoint = SmartDashboard.getNumber("Set Position", 0);
          m_leftPID.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
        processVariable = m_leftEncoder.getPosition();
        }
        
    }
    
    
    
        // This method will be called once per scheduler run

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void setReference(double setPoint) {
        m_leftPID.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);

    }
}   
