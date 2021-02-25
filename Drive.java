/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.*;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kTestDrive = "test generic";
  private static final String kTestAngle = "test angle";
  private static final String kTestAutonDrive = "test auton drive";
  private String m_autoSelected;
  public static double DEADZONE_VALUE_RIGHT = 0.1;
  public static double DEADZONE_VALUE_LEFT = -0.1;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  public CANSparkMax c1, c2;
  public CANEncoder e1, e2;
  public CANPIDController pc1, pc2;
  public ADXRS450_Gyro g1;
  public Joystick prim_joy;
  public Drive d;
  public static double P = 0.15;
  public static double I = 0.001;
  public static double D = 0;
  public static double inc_error= 0;
  public static boolean TOG_ROT = false;
  PIDControl anglePID;
  public static int dx[] = {1,0,-1,0};
  public static int dy[] = {0,1,0,-1};
  public static int angVal = 21;
  public static ArrayList<Location> obstacleList;
  public static ArrayList<Integer> setAngles;
  Profile pathProf;
  PathConstructor pathConst;
  boolean butTog = false;
  
  public static double deadzone(double signal){
    return (signal >= Robot.DEADZONE_VALUE_LEFT && signal <= Robot.DEADZONE_VALUE_RIGHT ? 0 : signal);
  }
  public static double deadzone_continuous(double signal){
    double ret = (1/ (1 + Math.pow(Math.E, -1 * signal)));
    return ret;
  }
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   * 
   * 
   */

   public Robot(){
    c2 = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
    c1 = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    pc1 = c1.getPIDController();
    pc2 = c2.getPIDController();
   }

   //this creates a rectangular path with its control points
   public void testRectangularLocations(){
     obstacleList.add(new Location(2160,0));
     obstacleList.add(new Location(2160,-2088));
     obstacleList.add(new Location(0,-2088));
     obstacleList.add(new Location(0,0));
   }
   

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Test generic", kTestDrive);
    m_chooser.addOption("Test angle", kTestAngle);
    m_chooser.addOption("Test auton drive", kTestAutonDrive);
    //SmartDashboard.putData("Auto choices", m_chooser);
    prim_joy = new Joystick(0);
    g1 = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
    anglePID = new PIDControl(P,I,D);
    pc1.setP(P);
    pc1.setI(I);
    pc1.setD(D);
    pc2.setP(P);
    pc2.setI(I);
    pc2.setD(D);
    obstacleList = new ArrayList<Location>();
    testRectangularLocations();
    for(Location l : obstacleList){
        System.out.println(l.x + " " + l.y);
    }
    setAngles = new ArrayList<Integer>();
    for(int i = 10; i <= 360; i+= 10){
      setAngles.add(i);
    }
    d = new Drive(prim_joy,c1,c2,g1,obstacleList);
    System.out.println("SKIP");
    pathProf = new Profile();
    pathConst = new PathConstructor(20, obstacleList, d);
    g1.calibrate();
    pathProf.iterate_profiles();
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
    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
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
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    /*
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
    */
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    d.drive();
    if(prim_joy.getRawButton(1)){
      d.angleUpdate(angVal,anglePID);
    }


    /*
    if(prim_joy.getRawButton(2)){
      angVal += 3;
      System.out.println("INCREMENT: "  + angVal);
    }
    */
    SmartDashboard.putString("ANGLE VALUE: " , Double.toString(g1.getAngle()));
    if(prim_joy.getRawButton(3)){
      d.modifiedDrive(pathProf);
    }
    
    if(prim_joy.getRawButton(4) && d.angle_ptr < setAngles.size()){
      d.moveToCertainAngles(setAngles);
    }
    
    if(prim_joy.getRawButton(2)){
      d.printInformation();
    }
    
    /*
    if(prim_joy.getRawButton(3) && d.angle_ptr < setAngles.size()){
        d.moveToCertainAngles(setAngles);
    }
    */


    //System.out.println("ANGLE: " + g1.getAngle());
    /*
    if(prim_joy.getRawButtonPressed(1)){
      c1.set(0.05);
    }
    */
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    c1.set(deadzone(prim_joy.getRawAxis(1)));
    c2.set(-deadzone(prim_joy.getRawAxis(1)));
  }

  public void disabledInit(){
      pathProf.resetLocation();
      d.obstacle_ptr = 0;
      d.cur_angle = 0;
      d.IS_MODULATING_ANGLE = false;
      System.out.println("RESET INIT HAS BEEN RAN");
  }
}
