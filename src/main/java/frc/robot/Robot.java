// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.ColorSensorV3.LEDCurrent;
import com.revrobotics.ColorSensorV3.LEDPulseFrequency;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {


  Joystick js = new Joystick(0);

  TalonFX liftMotor = new TalonFX(10,"rio");
  TalonFX shoulderMotor = new TalonFX(20,"rio");
  CANcoder shoulderEncoder = new CANcoder(20, "rio");

  TalonFX rotateMotor = new TalonFX(30, "rio");
  TalonFX rollerMotor = new TalonFX(31, "rio");
  TalonFX sideRollerMotor = new TalonFX(32, "rio");

  CANcoder rotateEncoder = new CANcoder(30, "rio");


  StatusSignal<Current> rotateCurrentSignal;
  StatusSignal<Angle> rotatePositionSignal;
  StatusSignal<Temperature> rotateTempSignal;

  StatusSignal<Current> outputCurrent;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    CANcoderConfiguration shoulderEncoderConfig = new CANcoderConfiguration();

    shoulderEncoderConfig.MagnetSensor.MagnetOffset = 0.1413;

    shoulderEncoder.getConfigurator().apply(shoulderEncoderConfig);

    TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();

    shoulderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shoulderConfig.Voltage.PeakForwardVoltage = 1.0;
    shoulderConfig.Voltage.PeakReverseVoltage = -2.0;
    shoulderConfig.CurrentLimits.StatorCurrentLimit = 13;
    shoulderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shoulderConfig.Feedback.SensorToMechanismRatio = 100;
    shoulderConfig.Slot0.kP = 120;
    shoulderConfig.Slot0.kS = 0.2;
    shoulderConfig.Slot0.kV = 2.2;
    shoulderConfig.Slot0.kA = 0.3;
    shoulderConfig.MotionMagic.MotionMagicAcceleration = 2.5;
    shoulderConfig.MotionMagic.MotionMagicCruiseVelocity = 0.8;

    shoulderMotor.getConfigurator().apply(shoulderConfig);
    shoulderMotor.setPosition(shoulderEncoder.getPosition().getValue());

    TalonFXConfiguration liftConfig = new TalonFXConfiguration();
    
    liftConfig.Voltage.PeakForwardVoltage = 2.5;
    liftConfig.Voltage.PeakReverseVoltage = -2.5;
    liftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    liftConfig.Feedback.SensorToMechanismRatio = 50.0;
    liftConfig.Slot0.kP = 35.0;
    liftConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    liftConfig.Slot0.kG = 0.25;
    liftConfig.CurrentLimits.StatorCurrentLimit = 20;
    liftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    liftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    liftMotor.getConfigurator().apply(liftConfig);
    liftMotor.setPosition(0.0);

    outputCurrent = liftMotor.getStatorCurrent();

    CANcoderConfiguration rotateEncoderConfiguration = new CANcoderConfiguration();
    rotateEncoderConfiguration.MagnetSensor.MagnetOffset = -0.186;
    rotateEncoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    rotateEncoder.getConfigurator().apply(rotateEncoderConfiguration);
    

    //rotate motor
    TalonFXConfiguration rotateConfig = new TalonFXConfiguration();
    
    rotateConfig.Voltage.PeakForwardVoltage = 0.5;
    rotateConfig.Voltage.PeakReverseVoltage = -0.5;
    rotateConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rotateConfig.Feedback.FeedbackRemoteSensorID = 30;
    //rotateConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    rotateConfig.Slot0.kP = 100.0;
    rotateConfig.Slot0.kD = 0.0;
    rotateConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    rotateConfig.Slot0.kG = 0.0;
    rotateConfig.CurrentLimits.StatorCurrentLimit = 20;
    rotateConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rotateConfig.Feedback.SensorToMechanismRatio = 100;
    rotateConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rotateMotor.getConfigurator().apply(rotateConfig);
    rotateMotor.setPosition(rotateEncoder.getAbsolutePosition().getValueAsDouble() - 0.25);

    rotateTempSignal = rotateMotor.getDeviceTemp();
    rotateTempSignal.setUpdateFrequency(100);

    rotateCurrentSignal = rotateMotor.getStatorCurrent();
    rotateCurrentSignal.setUpdateFrequency(100);
    rotatePositionSignal = rotateMotor.getPosition();
    rotatePositionSignal.setUpdateFrequency(100);

    //rollers
    TalonFXConfiguration rollerConfiguration = new TalonFXConfiguration();

    rollerConfiguration.Voltage.PeakForwardVoltage = 1.0;
    rollerConfiguration.Voltage.PeakReverseVoltage = -1.0;
    rollerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfiguration.CurrentLimits.StatorCurrentLimit = 20;
    rollerConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;

    rollerMotor.getConfigurator().apply(rollerConfiguration);

    rollerConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    sideRollerMotor.getConfigurator().apply(rollerConfiguration);
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
    BaseStatusSignal.refreshAll(outputCurrent);
    SmartDashboard.putNumber("outputCurrent", outputCurrent.getValueAsDouble());
    SmartDashboard.putNumber("liftPosition", liftMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("shoulderPosition", shoulderMotor.getPosition().getValueAsDouble());
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
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double x = js.getRawAxis(0);
    double DEAD_ZONE = 0.2;
    x = (Math.abs(x) > DEAD_ZONE)? 
            ((x > 0)? 
                ((x-DEAD_ZONE)/(1-DEAD_ZONE)) :
                ((x+DEAD_ZONE)/(1-DEAD_ZONE))
            )
            : 0;
    
    rotateMotor.setControl(new PositionVoltage(0));


    if(js.getRawButton(8)) {
      shoulderMotor.setControl(new MotionMagicVoltage(0.052));

    } else {
      shoulderMotor.setControl(new StaticBrake());
    }

    if(js.getRawButton(7) && shoulderEncoder.getAbsolutePosition().getValueAsDouble() > 0.05){
      //liftMotor.setControl(new VoltageOut(x*1.5));
      liftMotor.setControl(new PositionVoltage(0.6));
      //SmartDashboard.putNumber("voltageOut", x*2.5);
    } else {
      liftMotor.setControl(new PositionVoltage(0.1));
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    liftMotor.setControl(new NeutralOut());
  }

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
