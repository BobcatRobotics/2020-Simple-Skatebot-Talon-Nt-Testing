/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.lang.Thread;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  private final Joystick m_stick = new Joystick(0); // Code setup to use gamepad with botton on back
                                                    // set in the D position (not X position)
  private double leftStickRaw = 0.0; // Raw value from left stick
  private double rightStickRaw = 0.0; // Raw value from right stick
  private double leftStickBias = -0.007813; // Left stick normal bias from zero
  private double rightStickBias = -0.007813; // Right stick normal bias from zero
  private double flipStick = -1.0; // Use -1.0 to flip raw value from stick
  private double scaleStick = 0.3; // Use 1.0 for max speed, lower values to impose slower driving
  private double leftStick = 0.0; // Flipped and scaled value of left stick
  private double rightStick = 0.0; // Flipped and scaled value of right stick

  private WPI_TalonSRX leftMotor = new WPI_TalonSRX(7); // Instantiate left motor controller
  private WPI_TalonSRX rightMotor = new WPI_TalonSRX(8); // Instantiate right motor controller
  private double encoderEPR = 4096.0; // Talon mag encoder edges per rev of encoder

  private double maxMotorVoltage = 11.0; // Voltage to be used as max for motor controllers
  private boolean enableMaxMotorVoltage = true; // Flag to enable using the max voltage setting

  private double leftDiameter = 0.3229; // left wheel diameter
  private double leftRatio = 3.0; // left gear ratio
  private double leftDistanceRaw = 0.0; // left wheel distance in raw encoder units
  private double leftVelocityRaw = 0.0; // left wheel velocity in raw encoder units per 100msec
  private double leftDistance = 0.0; // left wheel distance in feet
  private double leftVelocity = 0.0; // left wheel velocity in feet/sec
  private double leftRPM = 0.0; // left wheel RPM

  private double rightDiameter = 0.3229; // right wheel diameter
  private double rightRatio = 5.0; // right gear ratio
  private double rightDistanceRaw = 0.0; // right wheel distance in raw encoder units
  private double rightVelocityRaw = 0.0; // right wheel velocity in raw encoder units per 100msec
  private double rightDistance = 0.0; // right wheel distance in feet
  private double rightVelocity = 0.0; // right wheel velocity in feet/sec
  private double rightRPM = 0.0; // right wheel RPM

  private double PI = 3.141592;

  private NetworkTableInstance ntInst; // to hold reference to default network table instance

  private int DPcount = 0; // Count of passes through Disabled Periodic
  private int TPcount = 0; // Count of passes through Teleop Periodic

  private double TS0 = 0.0; // Time Stamp variables (should be an array, one day)
  private double TS1 = 0.0;
  private double TS2 = 0.0;
  private double TS3 = 0.0;
  private double TS4 = 0.0;
  private double TS5 = 0.0;
  private double TS6 = 0.0;
  private double TS7 = 0.0;
  private double TS8 = 0.0;
  private double TS9 = 0.0;
  private double LV0 = 0.0; // Left velocity variables (should be an array, one day)
  private double LV1 = 0.0;
  private double LV2 = 0.0;
  private double LV3 = 0.0;
  private double LV4 = 0.0;
  private double LV5 = 0.0;
  private double LV6 = 0.0;
  private double LV7 = 0.0;
  private double LV8 = 0.0;
  private double LV9 = 0.0;
  private double RV0 = 0.0; // Right velocity variables (should be an array, one day)
  private double RV1 = 0.0;
  private double RV2 = 0.0;
  private double RV3 = 0.0;
  private double RV4 = 0.0;
  private double RV5 = 0.0;
  private double RV6 = 0.0;
  private double RV7 = 0.0;
  private double RV8 = 0.0;
  private double RV9 = 0.0;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    // Initialize Falcons to Factory Default
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    // Set sense of master motor output
    leftMotor.setInverted(true);
    rightMotor.setInverted(false);

    // Set motors controllers to coast mode
    leftMotor.setNeutralMode(NeutralMode.Coast);
    rightMotor.setNeutralMode(NeutralMode.Coast);

    // Set voltage compensation to keep things consistent as battery discharges
    leftMotor.configVoltageCompSaturation(maxMotorVoltage);
    leftMotor.enableVoltageCompensation(enableMaxMotorVoltage);
    rightMotor.configVoltageCompSaturation(maxMotorVoltage);
    rightMotor.enableVoltageCompensation(enableMaxMotorVoltage);

    // Setup sensors, set current positoin to 0.0 and choose phase if needed.
    leftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    leftMotor.setSelectedSensorPosition(0, 0, 0);
    leftMotor.setSensorPhase(false);
    rightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    rightMotor.setSelectedSensorPosition(0, 0, 0);
    rightMotor.setSensorPhase(false);

    // Config the peak and nominal outputs
    leftMotor.configNominalOutputForward(0, 0);
    leftMotor.configNominalOutputReverse(0, 0);
    leftMotor.configPeakOutputForward(1, 0);
    leftMotor.configPeakOutputReverse(-1, 0);
    rightMotor.configNominalOutputForward(0, 0);
    rightMotor.configNominalOutputReverse(0, 0);
    rightMotor.configPeakOutputForward(1, 0);
    rightMotor.configPeakOutputReverse(-1, 0);

    // Configure the velocity measurement period and window period
    leftMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
    leftMotor.configVelocityMeasurementWindow(1);
    rightMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
    rightMotor.configVelocityMeasurementWindow(1);

    // Hook to network tables
    ntInst = NetworkTableInstance.getDefault();

    // Set the network table to update faster
    ntInst.setUpdateRate(0.01);
  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    TPcount++;
    leftStickRaw = m_stick.getRawAxis(Joystick.AxisType.kY.value);
    rightStickRaw = m_stick.getRawAxis(Joystick.AxisType.kTwist.value);
    leftStick = flipStick * scaleStick * (leftStickRaw - leftStickBias);
    rightStick = flipStick * scaleStick * (rightStickRaw - rightStickBias);

    // Drive the motor controllers with the left and right stick commands
    leftMotor.set(ControlMode.PercentOutput, leftStick);
    rightMotor.set(ControlMode.PercentOutput, rightStick);

    // If the right bumper is pressed reset the encoder values
    if (m_stick.getRawButton(6)) {
      leftMotor.setSelectedSensorPosition(0, 0, 0);
      rightMotor.setSelectedSensorPosition(0, 0, 0);
    }

    // Read Talon Sensors and display values
    try {
      readTalonsAndShowValues();
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  /**
   * This function is called periodically during disabled mode.
   */
  @Override
  public void disabledPeriodic() {
    DPcount++;
    leftStickRaw = m_stick.getRawAxis(Joystick.AxisType.kY.value);
    rightStickRaw = m_stick.getRawAxis(Joystick.AxisType.kTwist.value);
    leftStick = flipStick * scaleStick * (leftStickRaw - leftStickBias);
    rightStick = flipStick * scaleStick * (rightStickRaw - rightStickBias);

    // If the right bumper is pressed reset the encoder values, and 8 follows 7
    if (m_stick.getRawButton(6)) {
      leftMotor.setSelectedSensorPosition(0, 0, 0);
      rightMotor.setSelectedSensorPosition(0, 0, 0);
    }

    // Read Talon Sensors and display values
    try {
      readTalonsAndShowValues();
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  // Simple method to put values on the SmartDashboard
  public void readTalonsAndShowValues() throws InterruptedException {
    //SmartDashboard.putNumber("left stick raw:", leftStickRaw);
    //SmartDashboard.putNumber("right stick raw:", rightStickRaw);
    SmartDashboard.putNumber("DPcount:", DPcount);
    SmartDashboard.putNumber("TPcount:", TPcount);
    SmartDashboard.putNumber("left stick:", leftStick);
    SmartDashboard.putNumber("right stick:", rightStick);
    TS0 = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("TS0:", TS0);
    leftDistanceRaw = leftMotor.getSelectedSensorPosition();
    leftDistance = PI * leftDiameter * leftDistanceRaw / (leftRatio*encoderEPR);
    rightDistanceRaw = rightMotor.getSelectedSensorPosition();
    rightDistance = PI * rightDiameter * rightDistanceRaw / (rightRatio*encoderEPR);
    leftVelocityRaw = leftMotor.getSelectedSensorVelocity();
    LV0 = leftVelocityRaw;
    SmartDashboard.putNumber("LV0:", LV0);
    leftRPM = leftVelocityRaw * 600.0/(leftRatio*encoderEPR);
    leftVelocity = PI*leftDiameter*leftRPM/60.0;
    rightVelocityRaw = rightMotor.getSelectedSensorVelocity();
    RV0 = rightVelocityRaw;
    SmartDashboard.putNumber("RV0:", RV0);
    rightRPM = rightVelocityRaw * 600.0/(rightRatio*encoderEPR);
    rightVelocity = PI*rightDiameter*rightRPM/60.0;
    SmartDashboard.putNumber("left distance (ft):", leftDistance);
    SmartDashboard.putNumber("left velocity (ft/sec):", leftVelocity);
    SmartDashboard.putNumber("left RPM:", leftRPM);
    SmartDashboard.putNumber("right distance (ft):", rightDistance);
    SmartDashboard.putNumber("right velocity (ft/sec):", rightVelocity);
    SmartDashboard.putNumber("right RPM:", rightRPM);

    // Try to read inside the 20msec loop we're in
    Thread.sleep(2);
    TS1 = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("TS1:", TS1);
    LV1 = leftMotor.getSelectedSensorVelocity();
    SmartDashboard.putNumber("LV1:", LV1);
    RV1 = rightMotor.getSelectedSensorVelocity();
    SmartDashboard.putNumber("RV1:", RV1);

    Thread.sleep(2);
    TS2 = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("TS2:", TS2);
    LV2 = leftMotor.getSelectedSensorVelocity();
    SmartDashboard.putNumber("LV2:", LV2);
    RV2 = rightMotor.getSelectedSensorVelocity();
    SmartDashboard.putNumber("RV2:", RV2);

    Thread.sleep(2);
    TS3 = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("TS3:", TS3);
    LV3 = leftMotor.getSelectedSensorVelocity();
    SmartDashboard.putNumber("LV3:", LV3);
    RV3 = rightMotor.getSelectedSensorVelocity();
    SmartDashboard.putNumber("RV3:", RV3);

    Thread.sleep(2);
    TS4 = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("TS4:", TS4);
    LV4 = leftMotor.getSelectedSensorVelocity();
    SmartDashboard.putNumber("LV4:", LV4);
    RV4 = rightMotor.getSelectedSensorVelocity();
    SmartDashboard.putNumber("RV4:", RV4);

    Thread.sleep(2);
    TS5 = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("TS5:", TS5);
    LV5 = leftMotor.getSelectedSensorVelocity();
    SmartDashboard.putNumber("LV5:", LV5);
    RV5 = rightMotor.getSelectedSensorVelocity();
    SmartDashboard.putNumber("RV5:", RV5);

    Thread.sleep(2);
    TS6 = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("TS6:", TS6);
    LV6 = leftMotor.getSelectedSensorVelocity();
    SmartDashboard.putNumber("LV6:", LV6);
    RV6 = rightMotor.getSelectedSensorVelocity();
    SmartDashboard.putNumber("RV6:", RV6);
    
    // flush data to network tables
    ntInst.flush();
  }
}