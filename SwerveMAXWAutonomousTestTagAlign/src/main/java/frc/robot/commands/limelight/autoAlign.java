// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.limelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.LimelightObject;

public class autoAlign extends Command {

  private final DriveSubsystem swerve;
  private final SlewRateLimiter xLimiter, yLimiter, giroLimiter;
  private final PIDController drivePID, strafePID, rotationPID;
  private final double driveOffset, strafeOffset, rotationOffset;
  // private final LimelightObject limelight;

  LimelightObject limelight = new LimelightObject();

  /** Creates a new autoAlign. */
  public autoAlign() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = DriveSubsystem.getInstance();

    //Limites para a aceleração e um movimento melhor do robo
    this.xLimiter = new SlewRateLimiter(DriveConstants.kDriveAccelerationLimiter);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kDriveAccelerationLimiter);
    this.giroLimiter = new SlewRateLimiter(DriveConstants.kRotationAccelerationLimiter);

    //Controles PID para se alinhar
    this.drivePID = new PIDController(limelightConstants.kPdrive, limelightConstants.kIdrive, limelightConstants.kDdrive);

    this.strafePID = new PIDController(limelightConstants.kPstrafe, limelightConstants.kIstrafe, limelightConstants.kDstrafe);

    this.rotationPID = new PIDController(limelightConstants.kProtation, limelightConstants.kIrotation, limelightConstants.kDrotation);

    this.driveOffset = 2.1;
    this.strafeOffset = -0.2;
    this.rotationOffset = 10.2;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CameraServer.startAutomaticCapture();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double velForward = 0;
    double velStrafe = 0;
    double velGiro = 0;

    if(limelight.getObjectIsSeen() == true){
      velForward = drivePID.calculate(limelight.getALimelight(), driveOffset);
      velStrafe = strafePID.calculate(limelight.getXLimelight(), strafeOffset);
      velGiro = -rotationPID.calculate(limelight.getYaw(), rotationOffset);
    } else if(limelight.getObjectIsSeen() == false){
      velForward = 0;
      velStrafe = 0;
      velGiro = 0;
    }

    // 2. Apply deadband
    velForward = Math.abs(velForward) > OIConstants.kDriveDeadband ? velForward : 0.0;
    velStrafe = Math.abs(velStrafe) > OIConstants.kDriveDeadband ? velStrafe : 0.0;
    velGiro = Math.abs(velGiro) > OIConstants.kDriveDeadband ? velGiro : 0.0;

    //3. Make the driving smoother
    velForward = xLimiter.calculate(velForward) * 3;
    velStrafe = yLimiter.calculate(velStrafe) * 3;
    velGiro = giroLimiter.calculate(velGiro) * 5;

    // 4. Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds;
         
    //Relative to robot
    chassisSpeeds = new ChassisSpeeds(velForward, velStrafe, velGiro);

    // 5. Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
 
    // 6. Output each module states to wheels
    swerve.setModuleStates(moduleStates);
         


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
