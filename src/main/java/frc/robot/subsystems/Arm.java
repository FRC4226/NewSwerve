
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

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class Arm extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private CANSparkMax armMotor;
private SparkMaxPIDController m_armPID;
private RelativeEncoder m_armEncoder;
public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    
    /**
    *
    */
    public Arm() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
armMotor = new CANSparkMax(32, MotorType.kBrushless);

    armMotor.restoreFactoryDefaults();  
    armMotor.setInverted(true);
    armMotor.setIdleMode(IdleMode.kBrake);
    m_armPID = armMotor.getPIDController();
    m_armEncoder = armMotor.getEncoder();

  // PID coefficients
  kP = 6e-5; 
  kI = 1e-6;
  kD = 0; 
  kIz = 0; 
  kFF = 0.0006; 
  kMaxOutput = 1; 
  kMinOutput = -1;
  maxRPM = 5700;

  // Smart Motion Coefficients
  maxVel = 800; // rpm
  maxAcc = 800;

  // set PID coefficients
  m_armPID.setP(kP);
  m_armPID.setI(kI);
  m_armPID.setD(kD);
  m_armPID.setIZone(kIz);
  m_armPID.setFF(kFF);
  m_armPID.setOutputRange(kMinOutput, kMaxOutput);

  int smartMotionSlot = 0;
  m_armPID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
  m_armPID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
  m_armPID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
  m_armPID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);



    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    @Override
    public void periodic() {
    
        double setPoint, processVariable;
        boolean mode = SmartDashboard.getBoolean("Mode", false);
        if(mode) {
          setPoint = SmartDashboard.getNumber("Set Velocity", 0);
          m_armPID.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
          processVariable = m_armEncoder.getVelocity();
        } else {
          setPoint = SmartDashboard.getNumber("Set Position", 0);
          /**
           * As with other PID modes, Smart Motion is set by calling the
           * setReference method on an existing pid object and setting
           * the control type to kSmartMotion
           */
          m_armPID.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
          processVariable = m_armEncoder.getPosition();
        }
        
        SmartDashboard.putNumber("SetPoint", setPoint);
        SmartDashboard.putNumber("Process Variable", processVariable);
        SmartDashboard.putNumber("Output", armMotor.getAppliedOutput()); 
      }


    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void setReference(double setPoint) {
      m_armPID.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
      
    }
}

