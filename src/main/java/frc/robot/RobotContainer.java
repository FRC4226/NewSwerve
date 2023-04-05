// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive2.auto.Autos;
import frc.robot.commands.swervedrive2.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive2.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive2.drivebase.TeleopDrive;
import frc.robot.subsystems.swervedrive2.SwerveSubsystem;
import java.io.File;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
// Driver Buttons
  private JoystickButton a_xBox_Driver;
  private JoystickButton b_xBox_Driver;
  private JoystickButton x_xBox_Driver;
  private JoystickButton y_xBox_Driver;
  private JoystickButton lb_xBox_Driver;
  private JoystickButton rb_xBox_Driver;
  private JoystickButton r_Stick_Button_xbox_Driver;
  private JoystickButton l_Stick_Button_xbox_Driver;
  private JoystickButton start_xBox_Driver;
  private JoystickButton reset_xBox_Driver;
// Co-Pilot Sr. Homie Richard
  private JoystickButton a_xBox_Richard;
  private JoystickButton b_xBox_Richard;
  private JoystickButton x_xBox_Richard;
  private JoystickButton y_xBox_Richard;
  private JoystickButton lb_xBox_Richard;
  private JoystickButton rb_xBox_Richard;
  private JoystickButton r_Stick_Button_xbox_Richard;
  private JoystickButton l_Stick_Button_xbox_Richard;
  private JoystickButton start_xBox_Richard;
  private JoystickButton reset_xBox_Richard;

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));
                                                                         
  public final Gripper m_gripper = new Gripper();
  public final Tower m_Tower = new Tower();
  public final Arm m_Arm = new Arm();
  
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandJoystick driverController = new CommandJoystick(4);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(0);
  XboxController RichardXbox = new XboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
                                                          // Applies deadbands and inverts controls because joysticks
                                                          // are back-right positive while robot
                                                          // controls are front-left positive
                                                          () -> (Math.abs(driverXbox.getLeftY()) >
                                                                 OperatorConstants.LEFT_Y_DEADBAND)
                                                                ? driverXbox.getLeftY() *-1 : 0,
                                                          () -> (Math.abs(driverXbox.getLeftX()) >
                                                                 OperatorConstants.LEFT_X_DEADBAND)
                                                                ? driverXbox.getLeftX() *-1 : 0,
                                                          () -> -driverXbox.getRightX(),
                                                          () -> -driverXbox.getRightY(),
                                                          false);

    AbsoluteFieldDrive closedFieldAbsoluteDrive = new AbsoluteFieldDrive(drivebase,
                                                                         () -> (Math.abs(driverXbox.getLeftY()) >
                                                                                OperatorConstants.LEFT_Y_DEADBAND)
                                                                               ? driverXbox.getLeftY() : 0,
                                                                         () -> (Math.abs(driverXbox.getLeftX()) >
                                                                                OperatorConstants.LEFT_X_DEADBAND)
                                                                               ? driverXbox.getLeftX() : 0,
                                                                         () -> driverXbox.getRawAxis(2), false);
    TeleopDrive simClosedFieldRel = new TeleopDrive(drivebase,
                                                    () -> (Math.abs(driverXbox.getLeftY()) >
                                                           OperatorConstants.LEFT_Y_DEADBAND)
                                                          ? driverXbox.getLeftY() : 0,
                                                    () -> (Math.abs(driverXbox.getLeftX()) >
                                                           OperatorConstants.LEFT_X_DEADBAND)
                                                          ? driverXbox.getLeftX() : 0,
                                                    () -> driverXbox.getRawAxis(2), () -> true, false, true);
    TeleopDrive closedFieldRel = new TeleopDrive(
        drivebase,
        () -> (Math.abs(driverController.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? driverController.getY() : 0,
        () -> (Math.abs(driverController.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? driverController.getX() : 0,
        () -> -driverController.getRawAxis(3), () -> true, false, true);

    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? closedAbsoluteDrive : closedFieldAbsoluteDrive);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

  //Driver Bindings
    rb_xBox_Driver = new JoystickButton(driverXbox, XboxController.Button.kRightBumper.value);
    rb_xBox_Driver.whileTrue(new Grip( m_gripper));
    lb_xBox_Driver = new JoystickButton(driverXbox, XboxController.Button.kLeftBumper.value);
    lb_xBox_Driver.whileTrue(new GripOut(m_gripper));
    a_xBox_Driver = new JoystickButton(driverXbox, XboxController.Button.kA.value);
    a_xBox_Driver.toggleOnTrue(new ArmPickup( m_Arm));
    new JoystickButton(driverXbox, XboxController.Button.kX.value).onTrue((new InstantCommand(drivebase::zeroGyro)));
    new JoystickButton(driverXbox, XboxController.Button.kStart.value).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
   // lt_xBox_Driver = new XboxControllerAxisButton(m_Controller, XboxController.Axis.kLeftTrigger.value);
   // lt_xBox_Driver.whileTrue(new GripOut(m_gripper));

   // rt_xBox_Driver = new XboxControllerAxisButton(m_Controller, XboxController.Axis.kRightTrigger.value);
   // rt_xBox_Driver.whileTrue(new Grip(m_gripper));

   //Richard Bindings
    a_xBox_Richard = new JoystickButton(RichardXbox, XboxController.Button.kA.value);
    a_xBox_Richard.toggleOnTrue(new TowerScore(m_Tower));
    b_xBox_Richard = new JoystickButton(RichardXbox, XboxController.Button.kB.value);
    b_xBox_Richard.toggleOnTrue(new ArmScore(m_Arm));
    x_xBox_Richard = new JoystickButton(RichardXbox, XboxController.Button.kX.value);
    x_xBox_Richard.toggleOnTrue(new TowerMidScore(m_Tower));
    y_xBox_Richard = new JoystickButton(RichardXbox, XboxController.Button.kY.value);
    y_xBox_Richard.toggleOnTrue(new TowerPickup(m_Tower));
    //rb_xBox_Richard = new JoystickButton(driverXbox, XboxController.Button.kRightBumper.value);
    //rb_xBox_Richard.whileTrue(new HighScore(m_Tower, m_Arm));
    //lb_xBox_Richard = new JoystickButton(driverXbox, XboxController.Button.kLeftBumper.value);
    //lb_xBox_Richard.whileTrue(new MidScore(m_Tower, m_Arm));

   //Base Bindings (driver)
   
    //new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return Autos.exampleAuto(drivebase);
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
