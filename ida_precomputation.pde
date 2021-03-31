//iterative deepening A*
import java.util.*;


class IDAComparator implements Comparator<Location>{
  public int compare(Location one, Location two){
    int a = 1;
    //int a = (one.known_distance+one.heuristic)<(two.known_distance+two.heuristic)?1:0;
    return a;
  }
}

class IDA{
    ArrayList<Location> impasse;
    HashMap<Location, Boolean> optimized_impasse;
    ArrayList<Location> BFS_Path;
    Location start;
    Location goal;
    HashMap<Location, Location> parent_pointers;
    
    public float euclidean_heuristic_function(Location one, Location two){
      float f = dist(one.x,one.y,two.x,two.y);
      return f;
    }
    
    public void mapconv(){
      for(Location l : impasse){
        optimized_impasse.put(l,true);
      }
    }
    

    public IDA(ArrayList<Location> impasse, Location start, Location goal){
      this.impasse = impasse;
      this.optimized_impasse = new HashMap<Location, Boolean>();
      this.start = start;
      this.goal = goal;
      this.parent_pointers = new HashMap<Location,Location>();
      this.BFS_Path = new ArrayList<Location>();
    }
    //start off with heuristic value = 0
    public ArrayList<Location> IDA_Algo(Location start){
      return new ArrayList<Location>(); 
    }
    
    public ArrayList<Location> backtrack(Location end){
      Location copy = new Location(end.x,end.y);
      ArrayList<Location> res = new ArrayList<Location>();
      while(copy != null){
        res.add(copy);
        end= parent_pointers.get(copy);
      }
      return res;
    }
    
    public ArrayList<Location> BFS(){
      Queue<Location> q = new LinkedList<Location>();
      q.add(this.start);
      float[] dx = {1,0,-1,0};
      float[] dy = {0,-1,0,1};
      ArrayList<Location> seen = new ArrayList<Location>();
      ArrayList<Location> ret = new ArrayList<Location>();
      while(q.size()>0){
        Location tp = q.poll();
        println("POINT: " + tp.x + " " + tp.y);
        for(int idx = 0; idx < dx.length; idx++){
          Location nxt = new Location(tp.x+dx[idx],tp.y+dy[idx]);
          if(!seen.contains(nxt) && !impasse.contains(nxt)){
            parent_pointers.put(nxt,tp);
            println("ADD: " + nxt.x + " " + nxt.y);
            seen.add(nxt);
            q.add(nxt);
          }
        }
      }
      ret = backtrack(this.goal);
      this.BFS_Path = ret;
      return ret;
    }
}
