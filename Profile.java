package frc.robot;
import java.util.*;
import edu.wpi.first.wpilibj.*;


//for the sake of this, assume that path will be precomputed by some separate process
//(translate processing code onto here)
public class Profile {
    public HashMap<Integer, Location> velocity_profile;
    public HashMap<Integer, Location> acceleration_profile;
    public HashMap<Integer, Double> angle_profile;
    public ArrayList<Location> path;
    public Location MY_CURRENT_POSITION;
    public static double DISTANCE_UPPER_THRESHOLD = 30;
    public static double DISTANCE_LOWER_THRESHOLD = 0;
    public Profile(ArrayList<Location> path){
        this.path = path;
        velocity_profile = new HashMap<Integer, Location>();
        acceleration_profile = new HashMap<Integer, Location>();
        angle_profile = new HashMap<Integer, Double>();
        MY_CURRENT_POSITION = new Location(0,0);
    }
    public void resetLocation(){
        MY_CURRENT_POSITION = new Location(0,0);
    }
    //assume that the time over wich velocity/acceleration changes is 1 unit
    public void constructVelocityMap(){
        if(path == null){
            return;
        }
        //PREDICATED ON SOME SEED STARTING POSITION
        Location cur_pos= new Location(0,0);
        for(int i = 0; i < path.size(); i++){
            Location vel = path.get(i).subtract(cur_pos);
            vel.x /= 6;
            vel.y /= 6;
            velocity_profile.put(i,vel);
            System.out.println("WAYPOINT INDEX: " + i  + "VELOCITY: " + vel.x + " " + vel.y);
            cur_pos= path.get(i);
        }
    }
    public void constructAccelerationMap(){
        if(path == null){
            return;
        }
        for(int i = 0; i < path.size()-1; i++){
            Location acc = velocity_profile.get(i+1).subtract(velocity_profile.get(i));
            acceleration_profile.put(i, acc);
        }
    }
    public void constructAngleMap(){
        if(path == null){
            return;
        }
        for(int i = 0; i < path.size() - 1; i++){
            double avl = Math.atan( (double)( path.get(i+1).x - path.get(i).x)/(double)(path.get(i+1).y - path.get(i).y) ) * ((double)(180)/(double)(Math.PI));
            angle_profile.put(i,avl);
            System.out.println("WAYPOINT INDEX: " + i + "ANGLE: " + avl);
        }
    }
    public void angle_dead_reckoning(){
        angle_profile.put(0,-90.0);
        angle_profile.put(1,-180.0);
        angle_profile.put(2,-270.0);
    }
    public void updateMyPosition(Location velocity, double angle){
        //System.out.println("VELOCITY: " + Location.magnitude(velocity));
        //System.out.println("ANGLE: " + angle);
        //System.out.println("INCREMENT Y: " +  Math.sin(angle * ( (double)(Math.PI)/(double)(180))) );
        //System.out.println("INCREMENT X: " + Math.cos(angle * ( (double)(Math.PI)/(double)(180)) ));
        //System.out.println("PREVIOUS X: " + MY_CURRENT_POSITION.x);
        //System.out.println("COSINE VALUE: " + Math.cos(angle * ( (double)(Math.PI)/(double)(180)) ) );
        //System.out.println("SINE VALUE: " +  Math.sin(angle * ( (double)(Math.PI)/(double)(180)) ) );
        MY_CURRENT_POSITION.x += Location.magnitude(velocity) * 0.09 *  Math.cos( (angle) * ( (double)(Math.PI)/(double)(180)) );
        MY_CURRENT_POSITION.y += Location.magnitude(velocity) * 0.09 * Math.sin( (angle) * ( (double)(Math.PI)/(double)(180)) );
        //System.out.println("PREVIOUS X: " + MY_CURRENT_POSITION.x);
    }
    public boolean isWithinThreshold(Location goal){
        //System.out.println("CURRENT DISTANCE: " + Location.distance(goal, MY_CURRENT_POSITION));
        return Location.distance(goal, MY_CURRENT_POSITION) >= DISTANCE_LOWER_THRESHOLD &&  Location.distance(goal, MY_CURRENT_POSITION) <= DISTANCE_UPPER_THRESHOLD;
    }
    
}
