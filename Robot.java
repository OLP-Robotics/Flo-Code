// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;



/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive myRobot;
  private XboxController xbox;
  
  private final WPI_TalonSRX talonMotorL1 = new  WPI_TalonSRX(1);
  private final WPI_TalonSRX talonMotorL2 = new WPI_TalonSRX(2);
  private final WPI_TalonSRX talonMotorR3 = new  WPI_TalonSRX(3);
  private final WPI_TalonSRX talonMotorR4 = new WPI_TalonSRX(4);
  //private final WPI_VictorSPX victorMotorR3 = new WPI_VictorSPX(3);
  //private final WPI_VictorSPX victorMotorR4 = new WPI_VictorSPX(4);
  private final MotorController leftMotors = new MotorControllerGroup(talonMotorL1, talonMotorL2);
  //private final MotorController rightMotors = new MotorControllerGroup(victorMotorR3,victorMotorR4);
  private final double leftDrive = 0;
  private final double rightDrive = 0;


  //private final MotorControllerGroup leftMotors = new MotorControllerGroup(talonMotorL1, talonMotorL2);


  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //m_rightMotor.setInverted(true);

    //m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
   // m_rightStick = new XboxController(0);
//victorMotorR3.setInverted(true);
//victorMotorR4.setInverted(true);
xbox = new XboxController(0);

myRobot = new DifferentialDrive(leftMotors, rightMotors);



  }

  @Override
  public void teleopPeriodic() {
    double leftDrive = Math.sqrt(Math.abs(xbox.getLeftY()));
    if (xbox.getLeftY()<0){
       leftDrive*=-1;
    }
    double rightDrive = Math.sqrt(Math.abs(xbox.getRightY()));
    if (xbox.getRightY()<0){
      rightDrive*=-1;
   }
    myRobot.tankDrive( leftDrive, rightDrive);
  }
}
