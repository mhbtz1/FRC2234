
class TangentBug extends Bug{
  ArrayList<Location> obstacles;
  float OPTIM_FACTOR = 3.2;
  //information to know when bug is navigating: the point it is navigating to, and the dist(x, nav) + dist(nav, goal)
  Location CURRENT_POINT_NAVIGATED_TO = null;
  float CURRENT_DIST = 1000000009;
  //some constants
  public static final float SHIFT_FAC = 0;
  public static final float DISTANCE_FOR_EQUALITY = 2;
  public static final float OPTIMIZER_FROM_PREV_BUG = 20;
  //for each obstacle in the obstacle list, we need to represent a pair of angles
  //which shows the angles for which the robot is looking to the interior
  public HashMap<Integer,Location> inaccessible_angles;
  
  
  public void setObstacles(ArrayList<Location> obstacles){
    this.obstacles = obstacles;
  }
  
  public void compute_inaccessible_angles(){
    Location dft = obstacles.get(0);
    float DEG2RAD= (float)(180)/(float)(PI);
    for(int i = 1; i < obstacles.size()-1; i++){
      float ang1 = atan( (float)(obstacles.get(i).y-dft.y)/(float)(obstacles.get(i).x-dft.x) ) * DEG2RAD;
      float ang2 = atan( (float)(obstacles.get(i+1).y-obstacles.get(i).y)/(float)(obstacles.get(i+1).x-obstacles.get(i).x) ) * DEG2RAD;
       inaccessible_angles.put(i,new Location(ang1,ang2));
       println("ANGLE ONE: " + ang1 +  " : " + " ANGLE TWO: " + ang2);
    }
  }
  
  ArrayList<Location> highlight_obstacles = new ArrayList<Location>();
  public TangentBug(float sensing_radius, ArrayList<Location> past_places, Location current_loc, Location goal_loc){
    super(sensing_radius,past_places,current_loc,goal_loc);
    obstacles = new ArrayList<Location>();
    inaccessible_angles = new HashMap<Integer,Location>();
    //controlPoints = createWriter("controlPoints.txt");
  }
  
  //ISSUE: point robot tends to get inside obstacles without realizing so, because the algorithm I am implementing specifies the BOUNDS of obstacles, not each point
  //fix: the closer a point is to some obstacle point, we'll just add some repellent force to prohibit it from entering the obstacle (the effects of this should go away with the sampling + Bezier curves)
  
  public float repellentHeuristic(){
   float CLOSEST_DIST = 1000000000;
   for(Location l : obstacles){
     CLOSEST_DIST = min(CLOSEST_DIST,dist(this.current_loc.x,this.current_loc.y,l.x,l.y)+dist(l.x,l.y,this.goal_loc.x,this.goal_loc.y));
   }
    return 0.0;
  }
  
  
  //returns angle values representing the points at which an obstacle is hit by some casted line (uses binary search + an angular sweep)
  
  
  
  public float does_hit(float angle){
    //perform a binary search along the line for the point at which the raycasted line (would potentially) hit the edge of some obsacle
    //probably not as optimal as it could be, but good enough
    float INC_FACTOR = 0.05;
    float ret = 1;
   for(float i = 0; i <= 1; i += INC_FACTOR){
     float nx = this.current_loc.x + (((this.sensing_radius/2) ) * i * cos(angle));
     float ny = this.current_loc.y + (((this.sensing_radius/2) ) * i * sin(angle));
     for(Location l : obstacles){
       if(dist(l.x,l.y,nx,ny) < DISTANCE_FOR_EQUALITY ){
         highlight_obstacles.add(l);
         ret=i;
       }
     }
   }
    return ret;
  }
  
  public ArrayList<PVector> angular_sweep(float DIV){
    ArrayList<PVector> tmp = new ArrayList<PVector>();
    for(float i = 0; i < 2 * PI; i += 0.02){
      float sx = this.current_loc.x + ( ((this.sensing_radius/DIV)-SHIFT_FAC) * cos(i));
      float sy = this.current_loc.y + ( ((this.sensing_radius/DIV)-SHIFT_FAC) * sin(i));
      //stroke(0,255,0);
      //line(this.current_loc.x,this.current_loc.y,sx,sy);
      //stroke(0,0,0);
      tmp.add(new PVector(i,does_hit(i)));
    }
    return tmp;
  }
  //returns a PVector representing <the angle at which the minima was found, the minima>
  public PVector final_angular_sweep(){
    PVector opt = new PVector(100000008, -1);
    for(float i = 0; i < 2 * PI; i += 0.02){
      float eins = leave_heuristic(i,12);
      if( min(opt.x, eins) == eins){
        opt.x = eins;
        opt.y = i;
      }
    }
    return opt;
  }
  

  public float leave_heuristic(float angle, float DIV){
    float v1 = does_hit(angle);
    stroke(0,0,255);
    noFill();
    ellipse(this.current_loc.x,this.current_loc.y,2*this.sensing_radius/DIV,2*this.sensing_radius/DIV);
    stroke(0,0,0);
    float p1 = this.current_loc.x + ( 2 * (this.sensing_radius/DIV) * v1 * cos(angle) );
    float p2 = this.current_loc.y + ( 2 * (this.sensing_radius/DIV) * v1 * sin(angle) );
    if(does_hit(angle)>=0.95){
      return dist(this.current_loc.x,this.current_loc.y, p1, p2) + dist(p1,p2,this.goal_loc.x,this.goal_loc.y);
    } else {
      return 1000000009;
    }
  }
  
  public boolean contains(ArrayList<Location> m, Location n){
    for(Location l : m){
      if(l.equals(n)){
        return true;
      }
    }
    return false;
  }
     //first idea: there is an emergent boundary following behavior with minimizing d(x,o) + d(o,g) 
    public void tangent_bug_path_planning(){
       for(Location l : obstacles){
             fill(0,0,0);
             ellipse(l.x,l.y,CIRC_RADIUS,CIRC_RADIUS);
       }
       highlight_obstacles.clear();
      stroke(0);
      noFill();
      ellipse(this.current_loc.x,this.current_loc.y,this.sensing_radius,this.sensing_radius);
      fill(255,0,0);
      noStroke();
      ellipse(this.current_loc.x, this.current_loc.y, 8, 8);
      fill(0,255,0);
      ellipse(this.goal_loc.x,this.goal_loc.y,8,8);
      fill(255);
      boolean b = abs(this.current_loc.x-this.goal_loc.x)<=5 && abs(this.current_loc.y-this.goal_loc.y)<=5;
      //println("TANGENT BUG ALGORITHM");
     
      if(!b && !draw_obstacle){
        controlPoints.println(bg.current_loc.x + ":" + bg.current_loc.y);
        //tbg.past_places.add(tbg.current_loc);
        float FAILSAFE = atan((this.goal_loc.y-this.current_loc.y)/(this.goal_loc.x-this.current_loc.x));
        float prev_opt_ang = atan((this.goal_loc.y-this.current_loc.y)/(this.goal_loc.x-this.current_loc.x));
        ArrayList<PVector> valid_angles = angular_sweep(2);
        controlPoints.println(this.current_loc.x+":"+this.current_loc.y);
        if(valid_angles.size() != 0){
          float relax_dist = 1000000000;
          if(CURRENT_POINT_NAVIGATED_TO != null){
            //relax_dist = dist(this.current_loc.x,this.current_loc.y,CURRENT_POINT_NAVIGATED_TO.x,CURRENT_POINT_NAVIGATED_TO.y)+dist(CURRENT_POINT_NAVIGATED_TO.x,CURRENT_POINT_NAVIGATED_TO.y,this.goal_loc.x,this.goal_loc.y);
          } 
           float SHIFT_SPACE = (float)(PI)/(float)(2);
           float opt_ang = prev_opt_ang;
           for(PVector info : valid_angles){
               float ag = (info.x) * (float)(180)/(float)(PI);
               //println("POTENTIAL ANGLE: " + ag);
                if(info.y >= 0.95 || info.y <= 0.15){
                  continue;
                } else {
                  float p1 = this.current_loc.x + (info.y * cos(info.x) * ((this.sensing_radius/2)-SHIFT_FAC) );
                  float p2 = this.current_loc.y + (info.y * sin(info.x) * ((this.sensing_radius/2)-SHIFT_FAC) );
                  float dist = dist(this.current_loc.x, this.current_loc.y, p1,p2) + dist(p1,p2,this.goal_loc.x,this.goal_loc.y);
                  if(CURRENT_POINT_NAVIGATED_TO != null && this.current_loc.equals(CURRENT_POINT_NAVIGATED_TO)){
                    tbg.past_places.add(CURRENT_POINT_NAVIGATED_TO);
                    //println("REACHED!");
                    CURRENT_POINT_NAVIGATED_TO=null;
                  }
                  if(tbg.isContained(new Location(p1,p2))){
                    continue;
                  }
                  if(min(dist,relax_dist)==dist){
                    relax_dist = dist;
                    opt_ang = info.x;
                    this.CURRENT_POINT_NAVIGATED_TO = new Location(p1,p2);
                    CURRENT_DIST = dist(this.current_loc.x,this.current_loc.y,CURRENT_POINT_NAVIGATED_TO.x,CURRENT_POINT_NAVIGATED_TO.y)+dist(CURRENT_POINT_NAVIGATED_TO.x,CURRENT_POINT_NAVIGATED_TO.y,this.goal_loc.x,this.goal_loc.y);
                  }
                }
           }
           if(CURRENT_POINT_NAVIGATED_TO != null){
             fill(0,255,255);
             ellipse(CURRENT_POINT_NAVIGATED_TO.x,CURRENT_POINT_NAVIGATED_TO.y,8,8);
           }
           println("--------------------------------------------------------");
           //check the case where some point in your sensing space is closer to the goal than the chosen CURRENT_POINT_NAVIGATED_TO
           PVector param = final_angular_sweep();
           if(min(relax_dist, param.x)==param.x){
             //opt_ang = param.y;
           }
           prev_opt_ang = opt_ang;
           println("OPTIMAL ANGLE: " + opt_ang);
           this.updateLocation(new PVector(this.sensing_radius * 0.05 * cos(opt_ang),this.sensing_radius * 0.05 * sin(opt_ang)));
        } else {
          this.updateLocation(new PVector(this.sensing_radius * 0.05 * cos(prev_opt_ang), this.sensing_radius * 0.05 * sin(prev_opt_ang)));
          println("OPTIMAL ANGLE: " + prev_opt_ang);
        }
      } 
      
      if(b){
        SET_OF_WAYPOINTS=true;
        println("TANGENT BUG HAS FOUND ENDPOINT!");
      }
      
      
      
    }
   
}
