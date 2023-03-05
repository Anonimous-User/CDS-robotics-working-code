// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  Timer timer = new Timer();

  XboxController c1 = new XboxController(0);
  XboxController c2 = new XboxController(1);
  CANSparkMax leftFront = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax rightFront = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax leftBack = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax rightBack = new CANSparkMax(3, MotorType.kBrushless);
  
  public static class DrivetrainConstants {
    private static double integralError = 0.0f; // Integral error
    private static double previousError = 0.0f; // Previous error
    // Constants & variables for balancing
    private static final double kProportionalGain = 0.03; // Proportional gain
    private static final double kIntegralGain = 0.00; // Integral gain
    private static final double kDerivativeGain = 0.00; // Derivative gain
    private static final double kToleranceDegrees = 1.0f; // Tolerance for gyro angle
    private static final double kMaxOutput = 0.5; // Maximum output
    private static final double kMinOutput = -0.5; // Minimum output
  }
  
  /*
  double ConversionFactor = 10;//check multiplier to have it be in meters or inches or smth

  DoubleSolenoid reach = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
  Value state = Value.kReverse;

  CANSparkMax arm = new CANSparkMax(5, MotorType.kBrushless);
  RelativeEncoder armEncoder = arm.getEncoder();
  
  CANSparkMax intake = new CANSparkMax(0, MotorType.kBrushless);
  RelativeEncoder intakeEncoder = intake.getEncoder();
  
  CANSparkMax elevator = new CANSparkMax(0, MotorType.kBrushless);
  RelativeEncoder elevatorEncoder = elevator.getEncoder();
  */
  static ADIS16470_IMU gyro = new ADIS16470_IMU();

  double speedMultiplier = 1;
  
  MecanumDrive mdrive = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    timer.restart();

    rightBack.restoreFactoryDefaults();
    rightBack.setSmartCurrentLimit(35);
    rightBack.setIdleMode(IdleMode.kBrake);
    rightBack.set(0);

    rightFront.restoreFactoryDefaults();
    rightFront.setSmartCurrentLimit(35);
    rightFront.setIdleMode(IdleMode.kBrake);
    rightFront.set(0);

    leftBack.restoreFactoryDefaults();
    leftBack.setSmartCurrentLimit(35);
    leftBack.setIdleMode(IdleMode.kBrake);
    leftBack.set(0);

    leftFront.restoreFactoryDefaults();
    leftFront.setSmartCurrentLimit(35);
    leftFront.setIdleMode(IdleMode.kBrake);
    leftFront.set(0);

    rightBack.setInverted(true);
    rightFront.setInverted(true);


    gyro.reset();
    gyro.calibrate();
    
    
    /*
    elevator.restoreFactoryDefaults();
    elevator.setSmartCurrentLimit(35);
    elevator.setIdleMode(IdleMode.kBrake);
    elevator.set(0);
    elevator.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    elevator.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    elevator.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 10);//check limit value
    elevator.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 10);//check limit value
    elevatorEncoder.setPosition(0);
    elevatorEncoder.setPositionConversionFactor(ConversionFactor);
    //elevatorEncoder.setVelocityConversionFactor(1000 / 60);


    arm.restoreFactoryDefaults();
    arm.setSmartCurrentLimit(20);
    arm.setIdleMode(IdleMode.kBrake);
    arm.set(0);
    arm.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    arm.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    arm.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 10);//check limit value
    arm.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 10);//check limit value
    armEncoder.setPosition(0);
    armEncoder.setPositionConversionFactor(ConversionFactor);
    //armEncoder.setVelocityConversionFactor(1000 / 60);


    intake.restoreFactoryDefaults();
    intake.setSmartCurrentLimit(15);
    intake.setIdleMode(IdleMode.kBrake);
    intake.set(0);
    intake.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    intake.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    intake.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 10);//check limit value
    intake.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 10);//check limit value
    intakeEncoder.setPosition(0);
    intakeEncoder.setPositionConversionFactor(ConversionFactor);
    //intakeEncoder.setVelocityConversionFactor(1000 / 60);


    reach.set(Value.kReverse);
    state = Value.kReverse;


    UsbCamera cam = CameraServer.startAutomaticCapture(0);
    cam.setResolution(283,160);
    */
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

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
    timer.reset();
    timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
/*
    //Platform balance attempt, must be straight and right in front
    if(timer.get()<0.3){
      driveMecanum(0, 1, 0);
    } else if(timer.get()<1.5){
      driveMecanum(0, 0, 0);
    } else if(timer.get()<2){
      driveMecanum(0, 1, 0);
    } else if(timer.get()<2.2){
      driveMecanum(0, -0.1, 0);
    } else{
      driveMecanum(0, 0, 0);
    }
 */

    //simple auto
    if(timer.get()<0.5){
      driveMecanum(0, -0.5, 0);
    } else if(timer.get()<2){
      driveMecanum(0, 1, 0);
    } else{
      driveMecanum(0, 0, 0);
    }
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    //driving
    driveMecanum(threshold(c1.getLeftX()), -threshold(c1.getLeftY()), threshold(c1.getRightX())*0.67);

    //mdrive.driveCartesian(-threshold(c1.getLeftY()), -threshold(c1.getLeftX()), threshold(c1.getRightX())*0.67);

    SmartDashboard.putNumber("angle y", getPitch());
    SmartDashboard.putNumber("angle x", gyro.getAngle());
    
    
    //balance
    if(c1.getBButton()){
      balanceOnStation();
    }

/* 
    if(c1.getXButton()){
      turn180();
    }

    //pneumatic
    if(c2.getBButton()){
        if(state==Value.kReverse){
            state = Value.kForward;
        } else{
            state=Value.kReverse;
        }
      reach.set(state);
    }

    //arm
    if(c2.getRightTriggerAxis()>0){
      arm.set(0.75);
    } else if(c2.getLeftTriggerAxis()>0){
      arm.set(-0.75);
    } else{
      arm.set(0);
    }

    //intake
    if(c2.getRightBumper()){
      intake.set(0.75);
    } else if(c2.getLeftBumper()){
      intake.set(-0.75);
    } else{
      intake.set(0);
    }
    
    //elevator
    if(c2.getYButton()){
      elevator.set(0.75);
    } else if(c2.getAButton()){
      elevator.set(-0.75);
    } else{
      elevator.set(0);
    }
     */
  
  }

  /*
    public void turn180(){
      timer.reset();
      gyro.reset();
      while(timer.get()<1&&gyro.getAngle()<180){
        driveMecanum(0, 0, 0.5);
      }
    }
  */
  
  public void driveMecanum(double xSpeed, double ySpeed, double zRot) {
    // Calculate the angle and magnitude of the joystick input
    double theta = Math.atan2(ySpeed, xSpeed);
    double power = Math.hypot(xSpeed, ySpeed);
    
    // Calculate the sine and cosine of the angle, offset by 45 degrees
    double sin = Math.sin(theta - Math.PI / 4);
    double cos = Math.cos(theta - Math.PI / 4);
    double max = Math.max(Math.abs(sin), Math.abs(cos));

    // Calculate the motor powers for each wheel based on the joystick input
    double leftFront = power * cos / max + zRot;
    double rightFront = power * sin / max - zRot;
    double leftRear = power * sin / max + zRot;
    double rightRear = power * cos / max - zRot;

    // Scale the motor powers if necessary to avoid exceeding the maximum power
    if ((power + Math.abs(zRot)) > 1) {
        leftFront /= power + Math.abs(zRot);
        rightFront /= power + Math.abs(zRot);
        leftRear /= power + Math.abs(zRot);
        rightRear /= power + Math.abs(zRot);
      }

    if(leftFront==0&&leftRear==0&&rightFront==0&&rightRear==0){
      this.leftFront.stopMotor();
      this.leftBack.stopMotor();
      this.rightBack.stopMotor();
      this.rightFront.stopMotor();
    } else{
      setMotorSpeeds(leftFront, rightFront, leftRear, rightRear);
    }

    // Set the motor speeds to achieve the desired movement
  }
  public void setMotorSpeeds(double fl, double fr, double bl, double br) {
    fl*=speedMultiplier;
    bl*=speedMultiplier;
    fr*=speedMultiplier;
    br*=speedMultiplier;

    if(fl>1){
      fl=1;
    } else if(fl<-1){
      fl=-1;
    }
    if(fr>1){
      fr=1;
    } else if(fr<-1){
      fr=-1;
    }
    if(bl>1){
      bl=1;
    } else if(bl<-1){
      bl=-1;
    }
    if(br>1){
      br=1;
    } else if(br<-1){
      br=-1;
    }

    leftFront.set(fl);
    rightFront.set(fr);
    leftBack.set(bl);
    rightBack.set(br);
  }
  public double threshold(double val){
    return Math.abs(val) > 0.2 ? val : 0;
  }

  

  public void balanceOnStation() {
    double angle = getPitch();
    double error = -angle;
    if (Math.abs(error) < DrivetrainConstants.kToleranceDegrees) {
      setMotorSpeeds(0, 0, 0, 0);
    }
    double output = DrivetrainConstants.kProportionalGain*error+DrivetrainConstants.kIntegralGain*DrivetrainConstants.integralError+DrivetrainConstants.kDerivativeGain*(error-DrivetrainConstants.previousError);
    output = Math.max(DrivetrainConstants.kMinOutput, Math.min(DrivetrainConstants.kMaxOutput, output));
    setMotorSpeeds(output, output, output, output);
    DrivetrainConstants.integralError += error;
    DrivetrainConstants.previousError = error;
  }
  public static double getPitch(){
    return Math.atan2(gyro.getAccelY(), gyro.getAccelZ()) * 180 / Math.PI;
  }
  
  

  

  



  
  
  
  
  
  
  
  @Override
  public void disabledInit() {}
  
  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
