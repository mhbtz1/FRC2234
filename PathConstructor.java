package frc.robot;
import java.util.*;
import edu.wpi.first.wpilibj.*;
import com.revrobotics.*;


public class PathConstructor {
    //assume that 
    double SENSING_RADIUS;
    ArrayList<Location> obstacles;
    Drive controlDrive;
    public PathConstructor(double SENSING_RADIUS, ArrayList<Location> obstacles, Drive controlDrive){
        this.SENSING_RADIUS = SENSING_RADIUS;
        this.obstacles = obstacles;
        this.controlDrive = controlDrive;
    }

    public boolean hasloc(Location loc){
        for(Location l : obstacles){
            if(l.equals(loc)){
                return false;
            }
        }
        return true;
    }
    public double locdist(Location one, Location two){
        return Math.sqrt(Math.pow(one.x - two.x, 2) + Math.pow(one.y - two.y, 2));
    }
    public boolean valid_intersection(Location one, Location two){
        return locdist(one,two) <= SENSING_RADIUS;
    }
    public PathConstructor(ArrayList<Location> obstacles){
        this.obstacles = obstacles;
    }
    //use tangent bug algo to construct raw path(assumes preprocessed space)
    public ArrayList<Location> naive_construct_candidate_points(Location seed, Location goal){
        Location cur_loc = new Location(seed.x,seed.y);
        double DIR_X = (double)((goal.y - seed.y)/(goal.x - seed.x))/(double)(5);
        double DIR_Y = 0.2;
        ArrayList<Location> ans = new ArrayList<Location>();
        while(!seed.equals(goal)){
            double global_min = 1000000000;
            double slope_x_one = ((goal.y-seed.y)/(goal.x-seed.x))/(double)(5);
            double slope_y_one = 0.2;
            Location target_next= (cur_loc.add(new Location(slope_x_one,slope_y_one)));
            for(Location l : obstacles){
                double slope_x = ((goal.y-l.y)/(goal.x-l.x))/(double)(5);
                double slope_y = 0.2;
                Location zwei = new Location( (int)(cur_loc.x + slope_x), (int)(cur_loc.y + slope_y) );
                Location drei = new Location(cur_loc.x + DIR_X, cur_loc.y + DIR_Y);
                if(valid_intersection(cur_loc, l)){
                    double cur_relax = Location.distance(cur_loc,l) + Location.distance(l,zwei);
                    if(Math.min(cur_relax, global_min) == cur_relax){
                        global_min = cur_relax;
                        target_next = new Location(cur_loc.x + slope_x, cur_loc.y + slope_y);
                    }
                }
            }
            ans.add(target_next);
            cur_loc.add(target_next);
        }
        return ans;
    }
    //sample points from tangent bugnav for better path
    public ArrayList<Location> better_contruct_candidate_points(Location seed, Location goal){
        return new ArrayList<Location>();
    }


    public void driveControlPoints(){
        for(int i = 0; i < obstacles.size(); i++){
            
        }
    }
}
