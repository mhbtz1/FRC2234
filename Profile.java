package frc.robot;
import java.util.*;
import java.io.*;
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
    public ArrayList<Location> true_waypoints;
    public ArrayList<Location> waypoints;
    public ArrayList<BezierProfile> bez;
    public boolean ANGLE_FILE_HAS_BEEN_INITIALIZED;
    public static final double ANGLE_SHIFTING_FACTOR = 0;
    public Profile(ArrayList<Location> path){
        this.path = path;
        velocity_profile = new HashMap<Integer, Location>();
        acceleration_profile = new HashMap<Integer, Location>();
        angle_profile = new HashMap<Integer, Double>();
        this.MY_CURRENT_POSITION = new Location(200,300);
        this.true_waypoints = new ArrayList<Location>();
        this.waypoints = new ArrayList<Location>();
        this.bez = new ArrayList<BezierProfile>();
        ANGLE_FILE_HAS_BEEN_INITIALIZED = false;
    }
    public Profile(){
        velocity_profile = new HashMap<Integer, Location>();
        acceleration_profile = new HashMap<Integer, Location>();
        angle_profile = new HashMap<Integer, Double>();
        this.MY_CURRENT_POSITION = new Location(200,300);
        this.true_waypoints = new ArrayList<Location>();
        this.waypoints = new ArrayList<Location>();
        this.bez = new ArrayList<BezierProfile>();
        ANGLE_FILE_HAS_BEEN_INITIALIZED = false;
    }
    public void resetLocation(){
        this.MY_CURRENT_POSITION = new Location(200,300);
    }
    //assume that the time over wich velocity/acceleration changes is 1 unit
    public void constructVelocityMap(){
        System.out.println("PATH SIZE: " + true_waypoints.size());
        //PREDICATED ON SOME SEED STARTING POSITION
        Location cur_pos= new Location(200,200);
        for(int i = 0; i < true_waypoints.size(); i++){
            Location vel = true_waypoints.get(i).subtract(cur_pos);
            vel.x /= 6;
            vel.y /= 6;
            velocity_profile.put(i,vel);
            System.out.println("WAYPOINT INDEX: " + i  + "VELOCITY: " + vel.x + " " + vel.y);
            cur_pos= true_waypoints.get(i);
        }
    }
    public boolean loc_contains(ArrayList<Location> targ, Location tst){
        for(Location l : targ){
          if(l.equals(tst)){
            return true;
          }
        }
        return false;
      }
    

    public ArrayList<Location> parseFile(){
        ArrayList<Integer> maintain_hashcodes = new ArrayList<Integer>();
        ArrayList<Location> ans = new ArrayList<Location>();
        //String[] r = loadStrings("controlPoints.txt");
        ArrayList<Location> remove_duplicates = new ArrayList<Location>();
        try{
            String s = Filesystem.getDeployDirectory() + "/controlPathBehavior.txt";
            BufferedReader r = new BufferedReader(new FileReader(s) );
            String inp = "";
            while( (inp = r.readLine() ) != null){
            double one = Double.parseDouble(inp.substring(0, inp.indexOf(":")));
            double two = Double.parseDouble(inp.substring(inp.indexOf(":")+1));
            //System.out.println(one + ":" + two);
            int hc = (new Location( (float)(one), (float)(two) ) ).hashCode();
            if(loc_contains(remove_duplicates, new Location( (float)(one), (float)(two) ) ) ){continue;}
                remove_duplicates.add(new Location( (float)(one), (float)(two) ) );
                ans.add(new Location( (float)(one), (float)(two) ) );
            //println("POINT: " + one + " " + two);
            }
            r.close();
        } catch(IOException e){
            e.printStackTrace();
        }
        return new ArrayList<Location>(remove_duplicates);
    }

    //check out some weird stalling points, the robot is following the path, yet at waypoint 26 it gets stuck on a straight path
    public void iterate_profiles(){
        //if(waypoints.size() == 0){
          waypoints = parseFile();
          System.out.println("SIZE: " + waypoints.size());
          int ITER = 200;
          for(int i = 0; i < waypoints.size()-ITER; i+= ITER){
              //println(waypoints.get(i).x + " " + waypoints.get(i).y);
              BezierProfile b = new BezierProfile(waypoints.get(i).x,waypoints.get(i).y,waypoints.get(i+ITER/4).x,waypoints.get(i+ITER/4).y,waypoints.get(i+ITER/2).x,waypoints.get(i+ITER/2).y,waypoints.get(i+ITER).x,waypoints.get(i+ITER).y);
              bez.add(b);
            }
        int idx = 0;
        for(BezierProfile b : bez){
          for(float j = 0; j <= 1.0; j += 0.04){
            System.out.println("POINT: " + idx  +  " " + b.return_x(j) + " " + b.return_y(j));
            true_waypoints.add(new Location(b.return_x(j), b.return_y(j)));
            ++idx;
          }
        }

        constructAngleMap();
        constructVelocityMap();
        ANGLE_FILE_HAS_BEEN_INITIALIZED=true;
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
        //try{ 
          //  String s = Filesystem.getDeployDirectory() + "/the_angles.txt";
            //BufferedWriter  r= new BufferedWriter( new FileWriter( s ) );
            for(int i = 0; i < true_waypoints.size() - 1; i++){
                double avl = Math.atan( (double)( true_waypoints.get(i+1).y - true_waypoints.get(i).y)/(double)(true_waypoints.get(i+1).x - true_waypoints.get(i).x) ) * ((double)(180)/(double)(Math.PI));
                double mult = 1.0;
             
                angle_profile.put(i, ( avl + (ANGLE_SHIFTING_FACTOR) ) );
                //System.out.println("ANGLE AT TIME: " + i +  " IS: "  + avl);
                //r.write(Double.toString(avl).toCharArray());
                System.out.println("WAYPOINT INDEX: " + i + "ANGLE: " + ( -1 * (avl+ANGLE_SHIFTING_FACTOR) ) );
            }
            //r.close();
        //} catch(Exception e){
        //    e.printStackTrace();
        //}
        System.out.println("ANGLES HAVE BEEN ASSIGNED");
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
        MY_CURRENT_POSITION.x += Location.magnitude(velocity) * 0.8  * Math.cos( (angle) * ( (double)(Math.PI)/(double)(180)) );
        MY_CURRENT_POSITION.y += Location.magnitude(velocity) * 0.8  * Math.sin( (angle) * ( (double)(Math.PI)/(double)(180)) );
        //System.out.println("PREVIOUS X: " + MY_CURRENT_POSITION.x);
    }
    public boolean isWithinThreshold(Location goal){
        //System.out.println("CURRENT DISTANCE: " + Location.distance(goal, MY_CURRENT_POSITION));
        return Location.distance(goal, MY_CURRENT_POSITION) >= DISTANCE_LOWER_THRESHOLD &&  Location.distance(goal, MY_CURRENT_POSITION) <= DISTANCE_UPPER_THRESHOLD;
    }
    
}


