class TangentBug{
  Bug stbug;
  ArrayList<Location> obstacles;
  float OPTIM_FACTOR = 3.2;
  public TangentBug(Bug stbug, ArrayList<Location> obstacles){
    this.stbug = stbug;
    this.obstacles = obstacles;
  }
  
  public void setObstacles(ArrayList<Location> obstacles){
    this.obstacles = obstacles;
  }
  //returns angle values representing the points at which an obstacle is hit by some casted line (uses binary search + an angular sweep)
  
  public float does_hit(float sx, float sy, float angle){
    //perform a binary search along the line for the point at which the raycasted line (would potentially) hit the edge of some obsacle
    //probably not as optimal as it could be, but good enough
    float lp = 0;
    float rp = 1;
    while(lp < rp){
      float md = lp + (rp-lp)/2.0;
      float nx = stbug.current_loc.x + (stbug.sensing_radius * md * cos(angle));
      float ny = stbug.current_loc.y + (stbug.sensing_radius * md * sin(angle));
      boolean hit = false;
      for(Location l : obstacles){
        if(dist(l.x,l.y,nx,ny)<=2){
          hit = true;
        }
      }
      if(!hit){
        rp = md + 1;
      } else {
        lp = md - 1;
      }
    }
    
    return rp;
  }
  
  public ArrayList<PVector> angular_sweep(){
    ArrayList<PVector> tmp = new ArrayList<PVector>();
    for(float i = 0; i < 2 * PI; i += 0.01){
      float sx = stbug.current_loc.x + (stbug.sensing_radius * cos(i));
      float sy = stbug.current_loc.y + (stbug.sensing_radius * sin(i));
      float slope = (sy - stbug.current_loc.y)/(sx - stbug.current_loc.x);
      tmp.add(new PVector(i,does_hit(sx,sy,slope)));
    }
    return tmp;
  }
    
    public void tangent_bug_path(){
      while(true){
        float prev_opt_ang = -1;
        ArrayList<PVector> valid_angles = angular_sweep();
        if(valid_angles.size() != 0){
           float relax_dist = 1000000000;
           float opt_ang = -1;
           for(PVector info : valid_angles){
                if(info.y >= 0.95){
                  continue;
                } else {
                  float p1 = stbug.current_loc.x + (info.y * cos(info.x) * stbug.sensing_radius);
                  float p2 = stbug.current_loc.y + (info.y * cos(info.x) * stbug.sensing_radius);
                  float dist = dist(stbug.current_loc.x, stbug.current_loc.y, p1,p2) + dist(p1,p2,stbug.goal_loc.x,stbug.goal_loc.y);
                  if(min(dist,relax_dist)==dist){
                    opt_ang = info.x;
                  }
                }
           }
           prev_opt_ang = opt_ang;
           stbug.updateLocation(new PVector(stbug.sensing_radius*cos(opt_ang),stbug.sensing_radius*sin(opt_ang)));
        } else {
          stbug.updateLocation(new PVector(stbug.sensing_radius*cos(prev_opt_ang), stbug.sensing_radius * sin(prev_opt_ang)));
        }
      }
    }
   
}
