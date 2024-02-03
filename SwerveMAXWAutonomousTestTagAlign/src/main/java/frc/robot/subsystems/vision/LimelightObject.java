// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.limelightOffsets;

public class LimelightObject extends SubsystemBase {

  private static LimelightObject instance;

  private boolean objectIsSeen = false;

  private double x;

  private double y;

  private double a;

  private double v;

   private double s; 

   private double yaw;

   private int pipelineNumber;

   private limelightOffsets offsets;

  /** Creates a new LimelightObject. */
  public LimelightObject() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Chama as tables da Limelight
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    //Pega a entry da pipeline
    table.getEntry("pipeline").setNumber(pipelineNumber);

    //get array of robot pose entries
    double[] robotPose = table.getEntry("botpose").getDoubleArray(new double[6]);

    //Get Limelight tx
    NetworkTableEntry tx = table.getEntry("tx");
    x = -tx.getDouble(0.0);

    //Get Limelight ty
    NetworkTableEntry ty = table.getEntry("ty");
    y = ty.getDouble(0.0);

    //Get Limelight tv
    NetworkTableEntry tv = table.getEntry("tv");
    v = tv.getDouble(0.0);   

    //Get Limelight ts
    NetworkTableEntry ts = table.getEntry("ts");
    s = ts.getDouble(0.0);

    //Get Limelight yaw
    yaw = -robotPose[5];

    //Convertendo o parametro de Limelight V de Int para Boolean 
    if (v > 0){
      objectIsSeen = true;
    } else {
      objectIsSeen = false;
    }

        SmartDashboard.putNumber("Yaw", yaw);

        SmartDashboard.putNumber("X", x);
        
        SmartDashboard.putNumber("S", s);
        
        SmartDashboard.putNumber("Y", y);
        
        SmartDashboard.putNumber("A", a);
        
        SmartDashboard.putBoolean("objetivo?", objectIsSeen);

  }

      /**
     * Void to set the number of the pipeline to use
     * @param alingToAprilTag if it will align to april tag
     * 
     * Se alignToAprilTag is True, o Pipeline é 0, se não é o Pipeline 1
     */
    public void alingToAprilTag(boolean alingToAprilTag){
      if(!alingToAprilTag){
          pipelineNumber = 0;      
      } else {
          pipelineNumber = 1;
      }
  }

  public double getXLimelight(){
    return x;
  }

  public double getYLimelight(){
    return y;
  }

  public double getALimelight(){
    return v;
  }

  public double getVLimelight(){
    return v;
  }

  public double getYaw(){
    return yaw;
  }

  public boolean getObjectIsSeen(){
    return objectIsSeen;
  }

  public static LimelightObject getInstance(){
    if(instance == null){
      instance = new LimelightObject();
    }
    return instance;
  }





}
