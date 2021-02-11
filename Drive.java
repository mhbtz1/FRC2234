package frc.robot;
import edu.wpi.first.wpilibj.*;
import com.revrobotics.*;
import java.util.*;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive{
    Joystick j1;
    CANSparkMax c1, c2;
    CANPIDController pc1, pc2;
    ADXRS450_Gyro g1;
    ArrayList<Location> waypoints;
    Profile p;
    int INTERNAL_COUNTER;
    boolean ANGLE_HANDLER = false;
    int angle_ptr;
    int obstacle_ptr;
    double cur_angle= 0;
    boolean IS_MODULATING_ANGLE = false;
    public Drive(Joystick j1, CANSparkMax c1, CANSparkMax c2, ADXRS450_Gyro g1){
        this.j1 = j1;
        this.c1 = c1;
        this.c2 =c2;
        this.g1 = g1;
        pc1 = c1.getPIDController();
        pc2 = c2.getPIDController();
        g1.calibrate();
        angle_ptr = 0;
        INTERNAL_COUNTER = 0;
        obstacle_ptr = 0;
    }
    public Drive(CANSparkMax c1, CANSparkMax c2, ADXRS450_Gyro g1){
        this.c1 = c1;
        this.c2 = c2;
        this.g1 = g1;
        g1.calibrate();
        angle_ptr = 0;
    }
    public Drive(Joystick j1, CANSparkMax c1, CANSparkMax c2, ADXRS450_Gyro g1, ArrayList<Location> waypoints){
        this.j1 = j1;
        this.c1 = c1;
        this.c2 = c2;
        this.g1 = g1;
        pc1 = c1.getPIDController();
        pc2 = c2.getPIDController();
        this.waypoints = waypoints;
        p = new Profile(waypoints);
        p.constructVelocityMap();
        //p.constructAccelerationMap();
        //p.constructAngleMap();
        p.angle_dead_reckoning();
        System.out.println("DRIVE INITALIZED");
        g1.calibrate();
        angle_ptr = 0;
    }

    public void drive(){
        c1.set( (Robot.deadzone(j1.getRawAxis(0) + j1.getRawAxis(1)))/4);
        c2.set( (Robot.deadzone(j1.getRawAxis(0) - j1.getRawAxis(1)))/4);
        //System.out.println("SET C1: " +  (Robot.deadzone(j1.getRawAxis(0) + j1.getRawAxis(1)))/4);
        //System.out.println("SET C2: " + (Robot.deadzone(j1.getRawAxis(0) - j1.getRawAxis(1)))/4);
    }

    public void modifiedDrive(){
        if(waypoints == null){
            return;
        }   

        //THIS MOVES FORWARD
        //pc1.setReference(1.5, ControlType.kVoltage);
        //pc2.setReference(-1.5, ControlType.kVoltage);
        //THIS MOVES BACKWARD
        //pc1.setReference(-1.5, ControlType.kVoltage);
        //pc2.setReference(1.5, ControlType.kVoltage);
        
        if(obstacle_ptr < Robot.obstacleList.size()){
            if(!IS_MODULATING_ANGLE){
                double mag1 = Location.magnitude(p.velocity_profile.get(obstacle_ptr)) * 8;
                if(!p.isWithinThreshold(Robot.obstacleList.get(obstacle_ptr))){
                    pc1.setReference(mag1/1208, ControlType.kVoltage);
                    pc2.setReference(-mag1/1208, ControlType.kVoltage);
                    p.updateMyPosition(p.velocity_profile.get(obstacle_ptr),cur_angle);
                    System.out.println("X POSITION: " + p.MY_CURRENT_POSITION.x);
                    System.out.println("Y POSITION: " + p.MY_CURRENT_POSITION.y);
                } else {
                    System.out.println("MODULATING ANGLE ON");
                    IS_MODULATING_ANGLE=true;
                }
            } else {
                //System.out.println("THRESHOLD HAS BEEN REACHED, MODULATING ANGLE.");
                if(angleUpdate(p.angle_profile.get(obstacle_ptr), new PIDControl(Robot.P, Robot.I, Robot.D))){
                    cur_angle = p.angle_profile.get(obstacle_ptr);
                    ++obstacle_ptr;
                    System.out.println("ANGLE MODULATION COMPLETE!");
                    IS_MODULATING_ANGLE=false;
                }
            }
        }




        /*
        if(INTERNAL_COUNTER < waypoints.size() - 1){
            if(angleUpdate(p.angle_profile.get(INTERNAL_COUNTER), new PIDControl(Robot.P, Robot.I, Robot.D))){
                if(Location.is_greater_or_equal(p.MY_CURRENT_POSITION, waypoints.get(INTERNAL_COUNTER))){
                    ++INTERNAL_COUNTER;
                } else {
                    Location absVel = p.velocity_profile.get(INTERNAL_COUNTER);
                    pc1.setReference( Location.magnitude(absVel), ControlType.kVoltage);
                    pc2.setReference( -Location.magnitude(absVel), ControlType.kVoltage);
                    p.updateMyPosition(absVel);
                }
            }
        }
        */
        
    }

    public void moveToCertainAngles(ArrayList<Integer> angles){
        if(angleUpdate(angles.get(angle_ptr), new PIDControl(Robot.P, Robot.I, Robot.D))){
            angle_ptr++;
        }
    }

    public double map_between_stuff(double value){
        double ans = (value - 0)/(0.5-0);
        return ans;
    }

    public boolean angleUpdate(double setpoint, PIDControl p){
        double error = p.P * (setpoint - g1.getAngle());
        //+p.I * p.INC_ERROR;
        /*
        pc1.setReference(error, ControlType.kVelocity);
        pc2.setReference(error, ControlType.kVelocity);
        */
        SmartDashboard.putString("TARGET: " , Double.toString(setpoint));
        SmartDashboard.putString("CURRENT: " , Double.toString(g1.getAngle()));
        double set_sparks = error * (double)(1)/(double)(28);
        c1.set(set_sparks);
        c2.set(set_sparks);
        SmartDashboard.putString("VALUE FOR SPARK TURNING: " , Double.toString(set_sparks));
        p.INC_ERROR += error;
        //System.out.println("SETPOINT: " + setpoint + "CURRENT ANGLE: " + g1.getAngle());
        if( Math.abs(g1.getAngle() - setpoint) <= 10){
            p.error_reset();
            //System.out.println("TRUE");
            return true;
        }
        //System.out.println("FALSE");
        return false;
    }

    public void printInformation(){
        for(int i = 0; i < Robot.obstacleList.size(); i++){
            Location l = Robot.obstacleList.get(i);
            if(i < Robot.obstacleList.size() - 1){
                double vel = Location.magnitude(p.velocity_profile.get(i));
                double ang = p.angle_profile.get(i);
                System.out.println("VELOCITY: " + vel);
                System.out.println("ANGLE: " + ang);
            }
            System.out.println("POSITION: " + l.x + " " + l.y);
        }
    }

    



}