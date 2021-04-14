//bug pathfinding algorithm presentation:
//https://spacecraft.ssl.umd.edu/academics/788XF14/788XF14L14/788XF14L14.pathbugsmapsx.pdf

import java.util.*;

//WLOG we will assume our bugs position (x,y)  <= our goal position (gx, gy), since we can consider our bug starting at the goal position and navigating to its "initial" position, in thtat case

class Bug{
  float sensing_radius;
  float original_radius;
  boolean MODULATE_RADIUS;
  ArrayList<Location> past_places = new ArrayList<Location>();
  ArrayList<Location> heuristic_path = new ArrayList<Location>();
  ArrayList<Location> obstacle_seen = new ArrayList<Location>();
  ArrayList<Integer> known_displacements = new ArrayList<Integer>();
  Location current_loc;
  Location goal_loc;
  public Bug(){
    
  }
  public Bug(float sensing_radius, ArrayList<Location> past_places, Location current_loc, Location goal_loc){
    this.sensing_radius = sensing_radius;
    this.past_places = past_places;
    this.current_loc = current_loc;
    this.goal_loc = goal_loc;
    this.MODULATE_RADIUS=false;
    this.original_radius = sensing_radius;
    known_displacements = new ArrayList<Integer>();
  }
  public boolean isContained(Location match){
    for(Location l : past_places){
      if(l.equals(match)){
        return true;
      }
    }
    return false;
  }
  public boolean detReseed(int WINDOW_SIZE){
    float mdiff = 0;
    if(WINDOW_SIZE >= past_places.size()){
      return false;
    }
    Location one = bg.past_places.get(bg.past_places.size() - WINDOW_SIZE);
    Location two = bg.past_places.get(WINDOW_SIZE - 1);
    return dist(one.x,one.y,two.x,two.y) <= 2.5;
  }
  public void updateLocation(PVector v){
    this.current_loc.x+=v.x;
    this.current_loc.y+=v.y;
  }
}


//we will simply draw shapes into the map to give us the obstacles in the space.
Bug bg;
TangentBug tbg;
MotionProfiler mr;
//Perfect_Preprocessing prcs;
PrintWriter controlPoints;
static final int CIRC_RADIUS = 8;
ArrayList<Location> bad_places = new ArrayList<Location>();
Location current_loc = new Location(200,300);
Location goal_loc = new Location(800,400);
boolean draw_obstacle = true;
boolean SET_OF_WAYPOINTS = false;
float OPT_X=-1,OPT_Y=-1;
float PREV_OPT_X=-1,PREV_OPT_Y=-1;
float PREV_X = -1;
float PREV_Y = -1;
float LET_IT_RUN = 15;//if we encounter a glitch where we reach the end of an obstacle, we run the more primitive bugnav algo for some number of iterations
boolean INITIATE_COOLDOWN = false;
float COOLDOWN = 0;
boolean USE_FIRST_HEURISTIC = false;


//use this for toggling different heuristics
boolean TANGENT_BUG = true;
boolean BUGNAV_ONE = false;
boolean BUGNAV_TWO = false;

public void setup(){
  size(1400,1400);
  frameRate(20);
  //using new() for the constructors makes it so the slope itself is not updating as the bug moves(we have to make a copy of it)
  bg = new Bug(18, new ArrayList<Location>(), new Location(current_loc.x,current_loc.y), new Location(goal_loc.x,goal_loc.y) );
  tbg = new TangentBug(120, new ArrayList<Location>(), new Location(current_loc.x,current_loc.y), new Location(goal_loc.x,goal_loc.y));
  String[] r = loadStrings("controlPoints.txt");
  if(!SET_OF_WAYPOINTS){
    controlPoints = createWriter("controlPoints.txt");
  } 
  mr = new MotionProfiler();
}



public float[] change_x = {0.5,0,-0.5,0};
public float[] change_y = {0,-0.5,0,0.5};

public float intersect_circles(Location one, Location two){
  return dist(one.x,one.y,two.x,two.y);
}
public boolean valid_intersection(Location one, Location two){
  return intersect_circles(one,two) <= (bg.sensing_radius)/2;
}

public boolean intersect_obstacle(){
  float ep = 2.8;
  for(Location l : bad_places){
    if(intersect_circles(l,bg.current_loc) <= bg.sensing_radius + CIRC_RADIUS - ep){
      return true;
    }
  }
  return false;
}

// the first bug algorithm we have is based on the following idea( and future bug algorithms as well):

//1. we want to take the motion which will let us get close to our goal
//2. if we find an obstacle, we want to navigate around the obstacle while we still get close to the goal
//3. straight line paths are always faster than curved paths
//4. Following along the line that joins point A to point B is always best (bc of triangle inequality), this has not been implemented




public boolean path_planning_two(Bug bg){
  background(0);
  for(Location l : bad_places){
    ellipse(l.x,l.y,CIRC_RADIUS,CIRC_RADIUS);
  }
  
  stroke(255);
  noFill();
  ellipse(bg.current_loc.x,bg.current_loc.y,bg.sensing_radius,bg.sensing_radius);
  fill(255,0,0);
  noStroke();
  ellipse(bg.current_loc.x, bg.current_loc.y, 8, 8);
  fill(0,255,0);
  ellipse(bg.goal_loc.x,bg.goal_loc.y,8,8);
  fill(255);
  boolean condition=bg.current_loc.x == bg.goal_loc.x && bg.current_loc.y == bg.goal_loc.y;
  float DEFAULT_DIR_X = -0.5 * (float)((bg.current_loc.x-bg.goal_loc.x)/(bg.current_loc.y-bg.goal_loc.y));
  float DEFAULT_DIR_Y= -0.5;
  float DIR_X= - 0.5 * (float)((bg.current_loc.x-bg.goal_loc.x)/(bg.current_loc.y-bg.goal_loc.y));
  float DIR_Y= - 0.5;
  float new_x = DIR_X + bg.current_loc.x;
  float new_y = DIR_Y + bg.current_loc.y;
  if(!condition && !draw_obstacle){
    float glob_min = 1000000000;
    for(Location l : bad_places){
      if(valid_intersection(l,bg.current_loc)){
        float pot_x = ((float)((bg.current_loc.y-l.y)/(bg.current_loc.x-l.x)));
        float pot_y = 1;
        Location rel = new Location(bg.current_loc.x + pot_x, bg.current_loc.y + pot_y);
        float pot_x_2 = -pot_x;
        float pot_y_2 = -pot_y;
        Location rel2 = new Location(bg.current_loc.x + pot_x_2, bg.current_loc.y + pot_y_2);
        float dist_relax= dist(l.x,l.y,bg.current_loc.x,bg.current_loc.y) + dist(rel.x,rel.y,bg.goal_loc.x,bg.goal_loc.y);
        float dist_relax_two = dist(l.x,l.y,bg.current_loc.x,bg.current_loc.y) + dist(rel2.x,rel2.y,bg.goal_loc.x,bg.goal_loc.y);
        glob_min = min(glob_min,dist_relax);
        glob_min = min(glob_min, dist_relax_two);
        //slope should be perpendicular to slope between current position and considered obstacle point
        if(glob_min == dist_relax){
          DIR_X = pot_x;
          DIR_Y= pot_y;
          //DIR_X = ceil(DIR_X);
        } else if(glob_min == dist_relax_two){
          DIR_X = pot_x_2;
          DIR_Y=  pot_y_2;
          //DIR_X = ceil(DIR_X);
        }
      }
    }
    
    println( "POSITION: " + bg.current_loc.x + " " + bg.current_loc.y);
    
   
    
   println( "SLOPE: " + DIR_X + " " + DIR_Y);
   bg.past_places.add(bg.current_loc);
   bg.current_loc.x -= DIR_X;
   bg.current_loc.y -= DIR_Y;
   bg.known_displacements.add( (int)(max( abs(bg.current_loc.x - PREV_X), abs(bg.current_loc.y - PREV_Y)) ) );
    PREV_X = bg.current_loc.x;
    PREV_Y = bg.current_loc.y;
    bg.heuristic_path.add(bg.current_loc);
    PREV_OPT_X = DIR_X;
    PREV_OPT_Y = DIR_Y;
    DIR_X = DEFAULT_DIR_X;
    DIR_Y = DEFAULT_DIR_Y;
    
    if(bg.goal_loc.x  ==  bg.current_loc.x  && bg.goal_loc.y  == bg.current_loc.y){
      SET_OF_WAYPOINTS = true;
    }
    
   if(bg.detReseed(50)){
      return false;
    }
    return true;
  }
  return true;
}

public void path_planning_one(){
  //background(0);
  for(Location l : bad_places){
    ellipse(l.x,l.y,CIRC_RADIUS,CIRC_RADIUS);
  }
  stroke(255);
  noFill();
  ellipse(bg.current_loc.x,bg.current_loc.y,bg.sensing_radius,bg.sensing_radius);
  fill(255,0,0);
  noStroke();
  ellipse(bg.current_loc.x, bg.current_loc.y, 8, 8);
  fill(0,255,0);
  ellipse(bg.goal_loc.x,bg.goal_loc.y,8,8);
  fill(255);
   boolean condition=bg.current_loc.x == bg.goal_loc.x && bg.current_loc.y == bg.goal_loc.y;
  if(!condition && !draw_obstacle){
    controlPoints.println(bg.current_loc.x + ":" + bg.current_loc.y);
    println("POSITION: " + bg.current_loc.x + " " + bg.current_loc.y);
    float glob_min = 1000000000;
    float OPT_X=-1,OPT_Y=-1;
    float PREV_OPT_X=-1,PREV_OPT_Y=-1;
    for(int i = 0; i < 4; i++){
      Location nw = new Location(bg.current_loc.x+change_x[i],bg.current_loc.y+change_y[i]);
      float tot_dist = dist(bg.current_loc.x+change_x[i], bg.current_loc.y+change_y[i], bg.goal_loc.x, bg.goal_loc.y);
      if(!intersect_obstacle()){
          glob_min=min(glob_min,tot_dist);
          if(glob_min==tot_dist){
            OPT_X=change_x[i];
            OPT_Y=change_y[i];
          }
      }
    }
      
    
    if(OPT_X==-1 && OPT_Y==-1){
      float dist_max = 0;
      for(Location l : bad_places){
        for(int i = 0; i < 4; i++){
          Location loc = new Location(bg.current_loc.x+change_x[i],bg.current_loc.y+change_y[i]);
          float sav = intersect_circles(l,loc);
          if(sav <= bg.sensing_radius){
            dist_max= max(dist_max,sav);
            float goal_dist = dist(bg.current_loc.x+change_x[i],bg.current_loc.y+change_y[i],bg.goal_loc.x,bg.goal_loc.y);
            glob_min = min(glob_min, goal_dist);
            if(dist_max==sav && glob_min==goal_dist){
              
              if(bg.isContained(new Location(bg.current_loc.x+change_x[i],bg.current_loc.y+change_y[i]))){
                continue;
              }
              
              if(is_reversed(OPT_X,OPT_Y,PREV_OPT_X,PREV_OPT_Y)){
                continue;
              }
              
              OPT_X = change_x[i];
              OPT_Y = change_y[i];
           }
         }
        }
      }
    }
    
    

    bg.current_loc.x += OPT_X;
    bg.current_loc.y += OPT_Y;
    println("CHOICE: " + OPT_X + " " + OPT_Y);
   if(bg.isContained(bg.current_loc)){
         INITIATE_COOLDOWN = true;
    }
    
    if(!INITIATE_COOLDOWN){
      bg.current_loc.x += OPT_X;
      bg.current_loc.y += OPT_Y;
    } else {
      println("LETTING IT RUN...");
      if(COOLDOWN < LET_IT_RUN){
        bg.current_loc.x += 0.5;
        ++COOLDOWN;
      } else {
        println("TERMINATING RUNNING...");
        INITIATE_COOLDOWN = false;
        COOLDOWN = 0;
      }
    }
    
    bg.past_places.add(bg.current_loc);
    PREV_OPT_X = OPT_X;
    PREV_OPT_Y = OPT_Y;
  }
  
  if(bg.current_loc.x == bg.goal_loc.x && bg.current_loc.y == bg.goal_loc.y){
    SET_OF_WAYPOINTS = true;
  }
}

boolean is_reversed(float OPT_X, float OPT_Y, float PREV_OPT_X, float PREV_OPT_Y){
  if(OPT_X != 0){
    if(OPT_X==PREV_OPT_X*-1){
      return true;
    }
  }
  if(OPT_Y != 0){
    if(OPT_Y==PREV_OPT_Y*-1){
      return true;
    }
  }
  return false;
}

public void draw(){

    if(!myRRT.rrtExploration()){
      myRRT.displayRRT(myRRT.seed);
      myRRT.reset();
      ida = new IDA(myRRT.graph, current_loc, goal_loc);
      //ida.IDA();
    }

   //path_planning_one();
   //if(!draw_obstacle){
     if(TANGENT_BUG){
       if(!SET_OF_WAYPOINTS){
         //println("TANGENT PATHING");
         tbg.tangent_bug_path_planning();
       }
     } else if(BUGNAV_ONE){  
       if(!SET_OF_WAYPOINTS){
         path_planning_one();
       }
     } else if(BUGNAV_TWO){
       if(!SET_OF_WAYPOINTS){
         if(!path_planning_two(bg)){
           bg.current_loc.x += 10;
        }
       }
     }
     
     if(SET_OF_WAYPOINTS){
       mr.iterate_profiles();
     }
   //}
   //path_planning_two();

}
public void mouseDragged(){
  if(draw_obstacle){
    println(mouseX + " " + mouseY);
    bad_places.add(new Location( (int)(mouseX), (int)(mouseY)));
    tbg.setObstacles(bad_places);
  }
}

public void keyPressed(){
  if(key==BACKSPACE){
    println("Pressed!");
    draw_obstacle=false;
    tbg.compute_inaccessible_angles();
  }
}
