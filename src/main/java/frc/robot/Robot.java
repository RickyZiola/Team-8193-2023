// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

class TalonFX_Servo {
  private final TalonFX talon;
  private final double encoderTicksPerDegree;
  public TalonFX_Servo(int deviceNumber, double encoderTicksPerDegree) {
      this.talon = new TalonFX(deviceNumber);
      this.encoderTicksPerDegree = encoderTicksPerDegree;

      // Configure the Talon FX for position control
      talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
      talon.setSensorPhase(false);
      talon.setInverted(false);
      talon.config_kF(0, 0);
      talon.config_kP(0, 0.1);
      talon.config_kI(0, 0.);
      talon.config_kD(0, 0.1);
      talon.config_IntegralZone(0, 0);
      talon.configClosedLoopPeakOutput(0, 1);
      talon.configAllowableClosedloopError(0, 0);
  }
  public void set(double positionDegrees) {
      // Convert the position in degrees to encoder ticks
      int positionTicks = (int) (positionDegrees * encoderTicksPerDegree * 13);
      // Set the position setpoint for the Talon FX
      talon.set(ControlMode.Position, positionTicks);
  }
}
class SwerveDrive {
  WPI_TalonFX fld, frd, bld, brd;
  TalonFX_Servo flt, frt, blt, brt;
  public SwerveDrive(TalonFX_Servo flt, WPI_TalonFX fld, TalonFX_Servo frt, WPI_TalonFX frd, TalonFX_Servo blt, WPI_TalonFX bld, TalonFX_Servo brt, WPI_TalonFX brd){
    this.flt = flt;
    this.fld = fld;
    this.frt = frt;
    this.frd = frd;
    this.blt = blt;
    this.bld = bld;
    this.brt = brt;
    this.brd = brd;
  }
  public void update(float strafe, float drive, float turn) {
    float fLeftD;
    float fLeftT;
    float fRightD;
    float fRightT;
    float bLeftD;
    float bLeftT;
    float bRightD;
    float bRightT;
    if (Math.sqrt((double)(strafe*strafe + drive*drive)) > .1) {
      turn *= 1.5;
      float driveRatio = drive / strafe;
      float strafeRatio = strafe / drive;
      if(driveRatio > 1) {
        driveRatio = 1;
      }else if (driveRatio < 0) {
        driveRatio = 0;
      }
      if(strafeRatio > 1) {
        strafeRatio = 1;
      }else if (strafeRatio < 0) {
        strafeRatio = 0;
      }
      double[] perpendicularVector = new double[] {-(1 / strafeRatio) * turn, turn};
      double[] frontWheelDirectionVector = new double[] {(strafeRatio * (Math.sqrt(drive*drive + strafe*strafe))) + perpendicularVector[0], (driveRatio * Math.sqrt(drive*drive+strafe*strafe)) + perpendicularVector[1]};
      double[] backWheelDirectionVector = new double[] {(strafeRatio * (Math.sqrt(drive*drive + strafe*strafe))) + -perpendicularVector[0], -(driveRatio * Math.sqrt(drive * drive + strafe * strafe)) + -perpendicularVector[1]};
      double frontWheelDirection = Math.atan2(frontWheelDirectionVector[1], frontWheelDirectionVector[0]);
      double backWheelDirection = Math.atan2(backWheelDirectionVector[1], backWheelDirectionVector[0]);
      double wheelPower = Math.sqrt(Math.pow(strafe, 2) + Math.pow(drive, 2));
      fLeftT = (float) (frontWheelDirection);
      fLeftD = (float) wheelPower;
      fRightT = (float) (frontWheelDirection);
      fRightD = (float) wheelPower;
      bLeftD = (float) (backWheelDirection);
      bLeftT = (float) wheelPower;
      bRightD = (float) (backWheelDirection);
      bRightT = (float) wheelPower;
    } else if (Math.abs(turn) >= 0.1) {
      turn /= 2;
      fLeftT = -45;
      fLeftD = turn;
      fRightT = 45;
      fRightD = -turn;
      bLeftT = 45;
      bLeftD = turn;
      bRightT = -45;
      bRightD = -turn;
    } else {
      fLeftT = 0;
      fLeftD = 0;
      fRightT = 0;
      fRightD = 0;
      bLeftT = 0;
      bLeftD = 0;
      bRightT = 0;
      bRightD = 0;
      drive = 0;
      turn = 0;
      strafe = 0;
    }
    flt.set(fLeftT);
    fld.set(fLeftD);
    frt.set(fRightT);
    frd.set(fRightD);
    blt.set(bLeftT);
    bld.set(bLeftD);
    brt.set(bRightT);
    brd.set(bRightD);
  }
}

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  WPI_TalonFX fld, frd, bld, brd;
  TalonFX_Servo flt, frt, blt, brt;
  CANSparkMax intakeMotor;
  WPI_TalonSRX arm1, arm2;
  Joystick joystick;
  SwerveDrive sd;
  double targPosX, targPosY = 0;
  @Override
  public void robotInit() {
    joystick = new Joystick(0);
    
    flt = new TalonFX_Servo(0, 2048/360);
    fld = new WPI_TalonFX(1);
    frt = new TalonFX_Servo(2, 2048/360);
    frd = new WPI_TalonFX(3);
    blt = new TalonFX_Servo(4, 2048/360);
    bld = new WPI_TalonFX(5);
    brt = new TalonFX_Servo(6, 2048/360);
    brd = new WPI_TalonFX(7);

    intakeMotor = new CANSparkMax(8, MotorType.kBrushless);

    arm1 = new WPI_TalonSRX(9);
    arm2 = new WPI_TalonSRX(10);
    
    sd = new SwerveDrive(flt, fld, frt, frd, blt, bld, brt, brd);

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(joystick.getPOV() != -1) {
    targPosX += Math.cos(joystick.getPOV());
    targPosY += Math.sin(joystick.getPOV());
    }
    if(targPosX > 59) {
      targPosX = 59;
    }
    float trim = (float) -joystick.getRawAxis(3) / 4 + .75f;
    System.out.println(trim);
    sd.update((float) joystick.getRawAxis(0) * trim, (float) joystick.getRawAxis(1) * trim, (float) -joystick.getRawAxis(2) * trim);
    if(joystick.getRawButton(0)) {
      intakeMotor.set(.5);
    }
    if(joystick.getRawButton(1)) {
      intakeMotor.set(-.5);
    }
    double[] reverseKinematics = this.reverseKinematics(targPosX, targPosY);
    arm1.set(ControlMode.Position, reverseKinematics[0] * 300 * (4096 / 360));  ///move the motor to the desired position. Gear ratio is 300 : 1, and encoder ticks per revolution is 4096
    arm2.set(ControlMode.Position, reverseKinematics[1] * 100 * (4096 / 360)); /// move the motor to the desired position. Gear ratio is 100 : 1, and encoder ticks per revolution is 4096
  }
  public double[] reverseKinematics(double x, double y) {

    // Define the lengths of the two arm segments
    double l1 = 10.0; // length of first segment in inches
    double l2 = 8.0; // length of second segment in inches
  
    // Calculate the distance from the origin to the desired position
    double d = Math.sqrt(x * x + y * y);
  
    // Calculate the angle between the first arm segment and the x-axis
    double theta1 = Math.atan2(y, x) - Math.atan2(Math.sqrt(d * d - l2 * l2), l2);
  
    // Calculate the angle between the two arm segments
    double theta2 = Math.acos((l1 * l1 + l2 * l2 - d * d) / (2 * l1 * l2));
  
    // Convert the angles to degrees and return them in an array
    double[] angles = {Math.toDegrees(theta1), Math.toDegrees(theta1 + theta2)};
    return angles;
  }
  
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
