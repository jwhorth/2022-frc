package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class Shooter_SUB extends SubsystemBase {
  WPI_TalonFX LeftSideFly = new WPI_TalonFX(0); //FIXME change all the numbers for the motor functions
  WPI_TalonFX RightSideFly = new WPI_TalonFX(0); 
  WPI_TalonFX Rotation = new WPI_TalonFX(0);
  WPI_TalonSRX Feedmotor = new WPI_TalonSRX(0);
  

  Joystick ButtonBoard = new Joystick(5);

  //double turretP = Constants.TURRET_P; FIXME
  //double turretD = Constants.TURRET_D;
  //PIDController turretPIDController = new PIDController(turretP, 0, turretD);



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


  boolean wasHomeFound = false;
  
 
  double flywheelP = .05;
  double flywheelI = 0;
  double flywheelD = 0;
  double flywheelF = 0.05;
  
  /** Creates a new Shooter. */
  public Shooter_SUB() {
    LeftSideFly.follow(RightSideFly);
    LeftSideFly.setInverted(InvertType.OpposeMaster);
    
    RightSideFly.config_kP(0, flywheelP);
    RightSideFly.config_kI(0, flywheelI);
    RightSideFly.config_kD(0, flywheelD);
    RightSideFly.config_kF(0, flywheelF);
    /*
    Rotation.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    Rotation.getAlternateEncoder(FeedbackDevice.CTRE_MagEncoder_Absolute), countsPerRev)
    Rotation.configFeedbackNotContinuous(true, 10); // important for absolute encoders not to jump ticks randomly
    */ //FIXME look up the new functions for CANcoder to the Spark Max
  }

  
//Turret Flywheels CMDs





public double Flywheelspeed() {
  return RightSideFly.getSelectedSensorVelocity();
}

public void spinFylywheel(double speed) {
  RightSideFly.set(speed);
}


public void setFlywheelVelocityControl(double rpm) {
  RightSideFly.set(ControlMode.Velocity, rpm);

}



// Turret Rotation CMDs
public void spinTurretMotor(double speed) {
  if (goLeft && speed < 0) {
    Rotation.set(speed);
  } else if (goRight && speed > 0) {
    Rotation.set(speed);
  } else {
    Rotation.set(0);
  }
}

//
public double turretDistFromHome() {
  return Math.abs(turretCurrentPos - turretHome);
}
  //
public double getTurretTicks() {
  return Rotation.getSelectedSensorPosition();  //FIXME Replace with the new functions for CANcoder
}
//
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


//Feed Motor CMDs
  public void startPASS() {
    Feedmotor.set(1);
  }
  //
  public void stopPASS() {
    Feedmotor.set(0);
  }
//



  // Tracking CMDs

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
/*
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

*/ //FIXME uncomment this stuff
public void updateLimelight() {
  table = NetworkTableInstance.getDefault().getTable("limelight");
  tableTx = table.getEntry("tx");
  tableTy = table.getEntry("ty");
  tableTv = table.getEntry("tv");
  tx = tableTx.getDouble(-1);
  ty = tableTy.getDouble(-1);
  tv = tableTv.getDouble(-1);
}
 
//
public boolean limelightSeesTarget() {
  return tv == 1;
}
//

public String isTarget() {
  if (limelightSeesTarget()) {
    return "SEES TARGET";
  }
  return "NO TARGET";
}
//
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    updateLimelight();
    //hardStopConfiguration();
    turretCurrentPos = Rotation.getSelectedSensorPosition();  //FIXME Replace with the new functions for CANcoder
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    System.out.println(Flywheelspeed());


    if(Flywheelspeed() > 19000){
      readyToFire = true;
    } else {
      readyToFire = false;
    }

  



if (ButtonBoard.getRawButton(4)){
  Rotation.set(.3);
} else if(ButtonBoard.getRawButton(5)) {
  Rotation.set(-.3);
} else {
  Rotation.set(0);
}




if(ButtonBoard.getRawButton(7)){
  setFlywheelVelocityControl(-19500);

} else {
  spinFylywheel(0);
}







  

if(Flywheelspeed() > 19000 && ButtonBoard.getRawButton(8)){
  startPASS();
} else{
  stopPASS();
}










  }
}
