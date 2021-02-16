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
    public Profile(ArrayList<Location> path){
        this.path = path;
        velocity_profile = new HashMap<Integer, Location>();
        acceleration_profile = new HashMap<Integer, Location>();
        angle_profile = new HashMap<Integer, Double>();
        MY_CURRENT_POSITION = new Location(0,0);
        this.true_waypoints = new ArrayList<Location>();
        this.waypoints = new ArrayList<Location>();
        this.bez = new ArrayList<BezierProfile>();
        ANGLE_FILE_HAS_BEEN_INITIALIZED = false;
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
            String s = "C:\\Users\\Core\\pathfinding_stuff-Imported\\src\\main\\java\\frc\\robot\\controlPathBehavior.txt";
            BufferedReader r = new BufferedReader(new FileReader(s) );
            String inp = "";
            while( (inp = r.readLine() ) != null){
            double one = Double.parseDouble(inp.substring(0, inp.indexOf(":")));
            double two = Double.parseDouble(inp.substring(inp.indexOf(":")+1));
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

    public void iterate_profiles(){
        if(waypoints.size() == 0){
          waypoints = parseFile();
          int ITER = 200;
          if(bez.size() == 0){
            for(int i = 0; i < waypoints.size()-ITER; i+= ITER){
              //println(waypoints.get(i).x + " " + waypoints.get(i).y);
              BezierProfile b = new BezierProfile(waypoints.get(i).x,waypoints.get(i).y,waypoints.get(i+ITER/4).x,waypoints.get(i+ITER/4).y,waypoints.get(i+ITER/2).x,waypoints.get(i+ITER/2).y,waypoints.get(i+ITER).x,waypoints.get(i+ITER).y);
              bez.add(b);
            }
          }
        }
        for(BezierProfile b : bez){
          for(float j = 0; j <= 1.0; j += 0.04){
            //println(b.return_x(j) + " " + b.return_y(j));
            true_waypoints.add(new Location(b.return_x(j), b.return_y(j)));
          }
        }
        if(!ANGLE_FILE_HAS_BEEN_INITIALIZED){
            constructAngleMap();
            ANGLE_FILE_HAS_BEEN_INITIALIZED=true;
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
    public HashMap<Integer,Double> constructAngleMap(){
        HashMap<Integer,Double> angle_profile = new HashMap<Integer,Double>();
        if(true_waypoints == null){
            return new HashMap<Integer,Double>();
        }
        try{ 
            String s = "C:\\Users\\Core\\pathfinding_stuff-Imported\\src\\main\\java\\frc\\robot\\the_angles.txt";
            BufferedWriter  r= new BufferedWriter( new FileWriter( s ) );
            for(int i = 0; i < true_waypoints.size() - 1; i++){
                double avl = Math.atan( (double)( true_waypoints.get(i+1).y - true_waypoints.get(i).y)/(double)(true_waypoints.get(i+1).x - true_waypoints.get(i).x) ) * ((double)(180)/(double)(Math.PI));
                angle_profile.put(i,avl);
                System.out.println("ANGLE AT TIME: " + i +  " IS: "  + avl);
                r.write(Double.toString(avl).toCharArray());
                System.out.println("WAYPOINT INDEX: " + i + "ANGLE: " + avl);
            }
            r.close();
        } catch(Exception e){
            e.printStackTrace();
        }
        System.out.println("ANGLES HAVE BEEN ASSIGNED");
        return angle_profile;
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

