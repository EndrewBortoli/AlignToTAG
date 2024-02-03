// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.auto.sDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.LimelightObject;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  
  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putData("Autonomous Mode", m_chooser);

    LimelightObject.getInstance();

    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    /*
     * Outra maneira de definir um autonomous default
     */

        // Register named commands
    NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
    NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
    NamedCommands.registerCommand("print hello", Commands.print("hello"));


    m_chooser.setDefaultOption("Null", null);

    m_chooser.addOption("sDrive", new sDrive(m_robotDrive));
    m_chooser.addOption("reto", new PathPlannerAuto("straight"));
    m_chooser.addOption("3 pieces auto", new PathPlannerAuto("3 notes"));
    m_chooser.addOption("2 pieces auto", new PathPlannerAuto("2 pieces"));
    m_chooser.addOption("Test 90 Degrees", new PathPlannerAuto("TestTurning09Degrees"));
    m_chooser.addOption("Test 180 Degrees", new PathPlannerAuto("TestTurning180Degrees"));
    m_chooser.addOption("S test", new PathPlannerAuto("S"));
    // SmartDashboard.putData("test01", new PathPlannerAuto("test01"));



    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getZ(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    new JoystickButton(m_driverController, Joystick.AxisType.kZ.value)
    .whileTrue(new RunCommand(() -> {
      double rotation = m_driverController.getRawAxis(Joystick.AxisType.kZ.value);
      m_robotDrive.rotate(rotation);
    }, m_robotDrive));
  }



  private void configureButtonBindings() {




    new JoystickButton(m_driverController, OIConstants.B)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    
    new JoystickButton(m_driverController, OIConstants.START)
          .whileTrue(new RunCommand(
              () -> m_robotDrive.zeroHeading(),
              m_robotDrive));

    
    
          
    

  }

      /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
 }



  }

