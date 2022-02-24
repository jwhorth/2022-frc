// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake_SUB extends SubsystemBase {
  WPI_TalonSRX intakeMotor = new WPI_TalonSRX(0);
  WPI_TalonSRX intakeDeploy = new WPI_TalonSRX(0);

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final Color kBlueTarget = new Color(0.25, 0.29, 0.45);
  private final Color kRedTarget = new Color(0.60, 0.32, 0.07);
  private final Alliance alliance;
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private ShuffleboardTab sorterTab;
  private NetworkTableEntry detectedColorGraphNT;
  private NetworkTableEntry detectedColorNT;
  private NetworkTableEntry proximityNT;


  /** Creates a new SUB_PickUp. */
  public Intake_SUB() {
    alliance = DriverStation.getAlliance();
    configureColorMatcher();
    configureShuffleBoard();
  }

  public void SetIntakeRollerspeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setIntakeDeploySpeed(double speed) {
    intakeDeploy.set(speed);
  }
/*
  public double getIntakeDeployPosition() {
    return intakeDeployEncoder.getEncoder().getPosition();
  }
*/
  private void configureColorMatcher() {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
  }

  private void configureShuffleBoard() {
    sorterTab = Shuffleboard.getTab("Sorter");
    proximityNT = sorterTab.add("Proximity", 0).getEntry();
    detectedColorNT = sorterTab.add("DetectedColor", "").getEntry();
    detectedColorGraphNT = sorterTab.add("DetectedColorGraph", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
    sorterTab.add("Sorter", this);
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

  public void sendProximityToShuffleBoard(double proximity) {
    proximityNT.setNumber(proximity);
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

  @Override
  public void periodic() {


    

  }
}
