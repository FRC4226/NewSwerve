package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ArmScore;
import frc.robot.commands.ArmStore;
import frc.robot.commands.Grip;
import frc.robot.commands.GripOut;
import frc.robot.commands.HighScore;
import frc.robot.commands.TowerScore;
import frc.robot.commands.TowerStore;
import frc.robot.commands.swervedrive2.auto.Autos;
import frc.robot.commands.ArmPickup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.swervedrive2.SwerveSubsystem;

import java.util.List;

public class TwoAutoLeft extends SequentialCommandGroup {
  public TwoAutoLeft(SwerveSubsystem drivebase, Tower m_Tower, Arm m_Arm, Gripper m_Gripper) {


    addCommands(
      new ScoreAuto(drivebase, m_Tower, m_Arm, m_Gripper),
      new WaitCommand(0.3).andThen(new TwicePieceDriveLeft(drivebase, m_Arm, m_Gripper)),
      new MidCubeAuto(m_Gripper, m_Tower));
      // new Grip(m_Gripper).withTimeout(3));
      

    
  }
}
