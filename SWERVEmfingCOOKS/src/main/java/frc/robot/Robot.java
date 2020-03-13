/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.drivetrain.SwerveCooks;
import frc.robot.systems.drivetrain.WheelDrive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  AHRS navX;

  public XboxController _driveController = new XboxController(0);

  /*
  Param 1- Angle motor CAN ID.
  Param 2- Speed motor CAN ID.
  Param 3- Analog IN of navX encoder.
  */
  private WheelDrive _backRight = new WheelDrive(0, 0, 0, 0.73, 0.0, 0.0);// actual port ID's of angle and speed motors																			// motors
	private WheelDrive _backLeft = new WheelDrive(0, 0, 1, 0.73, 0.0, 0.0);// (In that order)
	private WheelDrive _frontRight = new WheelDrive(0, 0, 2, 0.73, 0.0, 0.0);
  private WheelDrive _frontLeft = new WheelDrive(0, 0, 3, 0.73, 0.0, 0.0);
  
  private SwerveCooks _swerve = new SwerveCooks(_backRight, _backLeft, _frontRight, _frontLeft);

  NetworkTableEntry encoderValue;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    navX = new AHRS(SPI.Port.kMXP);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    _swerve.drive(
      _driveController.getRawAxis(1), 
      _driveController.getRawAxis(0), 
      _driveController.getRawAxis(4), 
      navX.getAngle() - 90, 
      _driveController.getStickButton(Hand.kLeft));

      if (_driveController.getBButton())
      {
        _swerve.zeroEncoders();
      }

      if (_driveController.getAButton())
      {
        navX.zeroYaw();
        System.out.print("navX Zeroed.");
      }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with t.3he Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    //m_autonomous.StartAutonomousTimer();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        //m_autonomous.Auton();
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
