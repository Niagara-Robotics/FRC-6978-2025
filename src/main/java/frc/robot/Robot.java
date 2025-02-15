// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

  TalonFX liftMotor = new TalonFX(60,"rio");

  StatusSignal<Current> outputCurrent;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    //rotate motor
    TalonFXConfiguration liftConfig = new TalonFXConfiguration();
    
    liftConfig.Voltage.PeakForwardVoltage = 12.5;
    liftConfig.Voltage.PeakReverseVoltage = -12.5;
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
    
    if(js.getRawButton(8)){
      liftMotor.setControl(new VoltageOut(x*2.5));
      //liftMotor.setControl(new PositionVoltage(0.0));
      SmartDashboard.putNumber("voltageOut", x*2.5);
    } else if(js.getRawButton(7)){
      //liftMotor.setControl(new VoltageOut(x*1.5));
      //liftMotor.setControl(new PositionVoltage(3.0));
      SmartDashboard.putNumber("voltageOut", x*2.5);
    } else {
      liftMotor.setControl(new StaticBrake());
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
