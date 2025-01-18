// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;


/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */

public class Robot extends TimedRobot {

  private final TalonSRX LFMotor = new TalonSRX(1);
  private final TalonSRX LBMotor = new TalonSRX(2);
  private final TalonSRX RBMotor = new TalonSRX(3);
  private final TalonSRX RFMotor = new TalonSRX(4);
  private static final double deadband = 0.1;

  private XboxController controller1;
  double LJYAxis = 0;
  double RJYAxis = 0;

  /** Called once at the beginning of the robot program. */
  public Robot() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    
  }

  @Override
  public void robotInit() {
    controller1 = new XboxController(0);

    RFMotor.setInverted(true);
    RBMotor.setInverted(true);
    RBMotor.follow(RFMotor);
    LBMotor.follow(LFMotor);
 }

  @Override
  public void teleopPeriodic() {

    LJYAxis = MathUtil.applyDeadband(controller1.getLeftY(), deadband);
    RJYAxis = MathUtil.applyDeadband(controller1.getRightY(), deadband);
    //LFMotor.set(TalonSRXControlMode.PercentOutput, 1);
    
    

    RFMotor.set(TalonSRXControlMode.PercentOutput, RJYAxis);
    LFMotor.set(TalonSRXControlMode.PercentOutput, LJYAxis);
    
  
  }
}

