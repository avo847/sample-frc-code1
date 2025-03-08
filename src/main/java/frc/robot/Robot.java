// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.PowerDistribution;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
// replace deprecated code
// import com.ctre.phoenix.motorcontrol.GroupMotorControllers;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private DifferentialDrive m_myRobot; // drive base
  private final PowerDistribution m_pdp = new PowerDistribution();

  private XboxController gamepadDrive;

  // TO DO: tagged for removal
  private double robotHeading;
  private DutyCycle aileronPWM; // left-right
  private DutyCycle elevationPWM; // forward-back
  private double drivePWM;
  private double directionPWM; 
  private double drive;
  private double direction; 

  // drive motors
  private WPI_TalonSRX leftMotorControllerCIM1;
  private WPI_TalonSRX leftMotorControllerCIM2;
  private WPI_TalonSRX rightMotorControllerCIM1;
  private WPI_TalonSRX rightMotorControllerCIM2;
  private MotorControllerGroup leftMotorGroup; // DEPRECATED
  private MotorControllerGroup rightMotorGroup;

  // TO DO: replace deprecated code
  /*
  private GroupMotorControllers leftMotorGroup1;
  private GroupMotorControllers rightMotorGroup1;
  */

  // additional motors; controllers present, motors not
  /* 
  private WPI_TalonSRX climbMotorCIM1;
  private WPI_TalonSRX climbMotorCIM2;
  private WPI_VictorSPX conveyorMotorCIM1;
  private WPI_VictorSPX conveyorMotorCIM2;
  private WPI_VictorSPX colorWheelDrive;
  private WPI_VictorSPX colorWheelArm;
  */
  private PigeonIMU pigeonIMU;
  private double [] pigeonIMUData;

  private double leftEncoderReading;
  private double rightEncoderReading;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // motor port initialization - 
    // TO DO: update numbers
    leftMotorControllerCIM1 = new WPI_TalonSRX(10);
    leftMotorControllerCIM2 = new WPI_TalonSRX(1);
    leftMotorGroup = new MotorControllerGroup(leftMotorControllerCIM1, leftMotorControllerCIM2);
    rightMotorControllerCIM1 = new WPI_TalonSRX(2);
    rightMotorControllerCIM2 = new WPI_TalonSRX(3);
    rightMotorGroup = new MotorControllerGroup(rightMotorControllerCIM1, rightMotorControllerCIM2);
    
    // replace deprecated code with MotorContollerGroup:
    /* 
    rightMotorGroup1 = new GroupMotorControllers();
    rightMotorGroup1.register(rightMotorControllerCIM1);
    rightMotorGroup1.register(rightMotorControllerCIM2);
    leftMotorGroup1 = new GroupMotorControllers();
    leftMotorGroup1.register(leftMotorControllerCIM1);
    leftMotorGroup1.register(leftMotorControllerCIM2);
    */

    // create a differential drive system using left and right motor groups
    m_myRobot = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

    // controller
    gamepadDrive = new XboxController(0);

    // TO DO: Remove if not in use
    aileronPWM = new DutyCycle(new DigitalInput(0));
    elevationPWM = new DutyCycle(new DigitalInput(1));

    // pigeon IMU
    pigeonIMU = new PigeonIMU(rightMotorControllerCIM2);
    pigeonIMUData = new double[3];
    pigeonIMU.setFusedHeading(70);
  }


  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // readings from Power Distribution Panel (PDP)
    double temperatureCelsius = m_pdp.getTemperature();
    double totalPower = m_pdp.getTotalPower();
    double totalEnergy = m_pdp.getTotalEnergy();
    SmartDashboard.putNumber("Temperature", temperatureCelsius);
    SmartDashboard.putNumber("Total Power", totalPower);
    SmartDashboard.putNumber("Total Energy", totalEnergy);

    // motor readings - speeds
    SmartDashboard.putNumber("leftMotor", leftMotorControllerCIM1.get());
    SmartDashboard.putNumber("rightMotor", rightMotorControllerCIM1.get());
    
    // IMU readings
    pigeonIMU.getYawPitchRoll(pigeonIMUData);
    robotHeading = pigeonIMU.getFusedHeading();  
    SmartDashboard.putNumber("Robot Heading",robotHeading);

    // Quaterion E4T encoder readings
    leftEncoderReading = leftMotorControllerCIM1.getSelectedSensorPosition();
    rightEncoderReading = rightMotorControllerCIM1.getSelectedSensorPosition();
    SmartDashboard.putNumber("left encoder", leftEncoderReading);
    SmartDashboard.putNumber("right encoder", rightEncoderReading);

    // PWM input readings
    drivePWM = (int) elevationPWM.getHighTimeNanoseconds()/1000;
    directionPWM = (int) aileronPWM.getHighTimeNanoseconds()/1000;
    SmartDashboard.putNumber("aileron time high", drivePWM);
    SmartDashboard.putNumber("elevation time high", directionPWM);
    drive = (drivePWM - 1500) / 500;
    direction = (directionPWM - 1500) / 500;
    SmartDashboard.putNumber("Drive", drive);
    SmartDashboard.putNumber("Direction", direction);

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
    double leftX = gamepadDrive.getLeftX()*0.5; // from left controller
    double leftY = gamepadDrive.getLeftY()*1.0;
    //drivePWM = (int) elevationPWM.getHighTimeNanoseconds()/1000;
    //drive = (drivePWM - 1500) / 500;
    //directionPWM = (int) aileronPWM.getHighTimeNanoseconds()/1000;
    //direction = (directionPWM - 1500) / 500;    
    // combine left controller on Windows PC and PWM input
    m_myRobot.arcadeDrive(-leftY, -leftX); // reversed inputs due to coord sys orientation
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

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
