package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.lang.model.util.ElementScanner6;
import javax.swing.plaf.TreeUI;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;

public class Shooter_SUB extends SubsystemBase {
  WPI_TalonFX Flywheel = new WPI_TalonFX(22); 
  WPI_TalonFX Rotation = new WPI_TalonFX(23);
  WPI_TalonSRX Feedmotor = new WPI_TalonSRX(24);
  

  Joystick ButtonBoard = new Joystick(5);

  double turretP = 0; //FIXME PIDS
  double turretD = 0;
  PIDController turretPIDController = new PIDController(turretP, 0, turretD);

  private DigitalInput ballCounter;

  public double turretCurrentPos;
  public double turretHome = 0; //FIXME change all the numbers for Turret Home and Stops.
  public double turretLeftStop = 0;
  public double turretRightStop = 0;

  boolean goLeft = true;
  boolean goRight = true;

  
  public NetworkTable table;
  NetworkTableEntry tableTx, tableTy, tableTv;
  double tx, ty, tv;

  public boolean readyToFire;

  public static int ballCount;
  public static double disspeed; //FIXME define disspeed using the trig thing
  public static double distanceSpeed;
  public static double testingrpm;
  

  boolean wasHomeFound = false;


  
 
  double flywheelP = .35;
  double flywheelI = 0;
  double flywheelD = 0;
  double flywheelF = 0.05;
  
  private final I2C.Port i2cPort = I2C.Port.kMXP;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final Color kBlueTarget = new Color(0.25, 0.29, 0.45);
  private final Color kRedTarget = new Color(0.60, 0.32, 0.07);
  private final Alliance alliance;
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private NetworkTableEntry detectedColorGraphNT;
  private NetworkTableEntry detectedColorNT;
  

  public NetworkTableEntry hits;
  public NetworkTableEntry miss;


  /** Creates a new Shooter. */
  public Shooter_SUB() {
    configureColorMatcher();
    alliance = DriverStation.getAlliance();
    
    Flywheel.config_kP(0, flywheelP);
    Flywheel.config_kI(0, flywheelI);
    Flywheel.config_kD(0, flywheelD);
    Flywheel.config_kF(0, flywheelF);

    ballCounter = new DigitalInput(0);  //FIXME CAN IDs NEED TO BE CHANGED.
    
    
    Rotation.getSelectedSensorPosition();
    //Rotation.configFeedbackNotContinuous(true, 10); //FIXME uncomment if needed for some reason
     //FIXME look up the new functions for CANcoder to the Spark Max
  }


//Ball Math
public void ZeroCount(){
  if (ballCount < 0){
    ballCount = 0;
  }
}

public boolean getBallCounter(){
  return ballCounter.get();
}  

public void BallMath(){
  if (getBallCounter() == false){
    ballCount ++;
  }
}


//Turret Flywheels 

public double FlyWheelspeed() {
  return Flywheel.getSelectedSensorVelocity();
}

public void SpinFlywheel(double speed) {
  Flywheel.set(speed);
}

public void SetFlywheelVelocityControl(double rpm) {
  Flywheel.set(ControlMode.Velocity, rpm);

}



// Turret Rotation
public void spinTurretMotor(double speed) {
  if (goLeft && speed < 0) {
    Rotation.set(speed);
  } else if (goRight && speed > 0) {
    Rotation.set(speed);
  } else {
    Rotation.set(0);
  }
}

public double turretDistFromHome() {
return Math.abs(turretCurrentPos - turretHome);
}

public double getTurretTicks() {
  return Rotation.getSelectedSensorPosition();  //FIXME Replace with the new functions for CANcoder
}

public void hardStopConfiguration() {   //FIXME Replace with the new functions for CANcoder

  if (Rotation.getSelectedSensorPosition() > turretRightStop) {
    // turretTalon.configPeakOutputReverse(0, 10);
    goRight = false;
  } else {
    // turretTalon.configPeakOutputReverse(-1, 10);
    goRight = true;
  }
  if (Rotation.getSelectedSensorPosition() < turretLeftStop) {
    // turretTalon.configPeakOutputForward(0, 10);
    goLeft = false;
  } else {
    // turretTalon.configPeakOutputForward(1, 10);
    goLeft = true;
  }
}


//Feed Motor
public void feedMotorSpeed(double speed){
  Feedmotor.set(speed);
}

//Tracking

public void goHome() {
    if ((turretCurrentPos > turretHome) && (turretCurrentPos - turretHome > 50)) {
      // If you're to the right of the center, move left until you're within 50 ticks (turret deadband)
      spinTurretMotor(0.3);
    } else if ((turretCurrentPos < turretHome) && (turretCurrentPos - turretHome < -50)) {
      // If you're to the left of the center, move right until you're within 50 ticks
      spinTurretMotor(-0.3);
    } else {
      spinTurretMotor(0);
    }
  }

public void track() {
    if (limelightSeesTarget()) {
      double heading_error = -tx + 0.5; // in order to change the target offset (in degrees), add it here
      // How much the limelight is looking away from the target (in degrees)
  
      double steering_adjust = turretPIDController.calculate(heading_error);
      // Returns the next output of the PID controller (where it thinks the turret should go)
      
      double xDiff = 0 - steering_adjust;
      double xCorrect = 0.05 * xDiff;
      spinTurretMotor(xCorrect);
    } else {
      goHome();
    }
  }

public void updateLimelight() {
  table = NetworkTableInstance.getDefault().getTable("limelight");
  tableTx = table.getEntry("tx");
  tableTy = table.getEntry("ty");
  tableTv = table.getEntry("tv");
  tx = tableTx.getDouble(-1);
  ty = tableTy.getDouble(-1);
  tv = tableTv.getDouble(-1);
}
 
public boolean limelightSeesTarget() {
  return tv == 1;
}

public String isTarget() {
  if (limelightSeesTarget()) {
    return "SEES TARGET";
  }
  return "NO TARGET";
}

//Color Stuff
private void configureColorMatcher() {
  m_colorMatcher.addColorMatch(kBlueTarget);
  m_colorMatcher.addColorMatch(kRedTarget);
}

public boolean isMyAllianceColor(Color color) {
  return (color == kBlueTarget && Alliance.Blue == alliance)
      || (color == kRedTarget && Alliance.Red == alliance);
}

public boolean isOpposingAllianceColor(Color color) {
  return (color == kRedTarget && Alliance.Blue == alliance)
      || (color == kBlueTarget && Alliance.Red == alliance);
}

public Color matchColor() {
  Color detectedColor = m_colorSensor.getColor();
  ColorMatchResult colorMatchResults = m_colorMatcher.matchColor(detectedColor);
  if (colorMatchResults == null) {
    return new Color(0, 0, 0);
  } else {
    return colorMatchResults.color;
  }
}

public double getColorSensorProxmity() {
  return m_colorSensor.getProximity();
}

public void sendDetectedColorToShuffleBoard(Color color) {
  if (color == kRedTarget) {
    detectedColorNT.setString("Red");
    detectedColorGraphNT.setNumber(1);
  } else if (color == kBlueTarget) {
    detectedColorNT.setString("Blue");
    detectedColorGraphNT.setNumber(-1);
  } else {
    detectedColorNT.setString("Null");
    detectedColorGraphNT.setNumber(0);
  }
}
private void configureShuffleBoard() {
  ShuffleboardLayout layout = Constants.PRIMARY_TAB.getLayout("Sorter", BuiltInLayouts.kList).withSize(2, 3);
  layout.add("Sorter command", this);
  detectedColorNT = layout.add("Detected color", "").getEntry();

  detectedColorGraphNT = Constants.SORTER_TAB.add("DetectedColorGraph", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
  hits = Constants.SORTER_TAB.add("Hit", 0).getEntry();
  miss = Constants.SORTER_TAB.add("Miss", 0).getEntry();
}




//
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    updateLimelight();
    //hardStopConfiguration();
    turretCurrentPos = Rotation.getSelectedSensorPosition();  //FIXME Replace with the new functions for CANcoder
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    BallMath();
    SmartDashboard.putNumber("Current Turret Encoder Position", getTurretTicks());
    SmartDashboard.putNumber("Flywheel RPM", FlyWheelspeed());
    SmartDashboard.putNumber("Ball Count", ballCount);
    SmartDashboard.putBoolean("Ready to Fire?", readyToFire);
  

  

    if(FlyWheelspeed() >= 19000){ 
      readyToFire = true;
    } else {
      readyToFire = false;
    }
  }
}


