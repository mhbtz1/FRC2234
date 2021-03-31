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
    HashMap<Integer, Boolean> optimized_impasse;
    
    //stores a list of integers which encode the positions they represent
    ArrayList<Integer> BFS_Path;
    Location start;
    Location goal;
    HashMap<Integer, Integer> parent_pointers;
    public int hashCode(float x, float y){
      int hash = (int)( 1400*(y) + x );
      //print("HASH:" + hash); 
      return hash;
    }
    public float euclidean_heuristic_function(Location one, Location two){
      float f = dist(one.x,one.y,two.x,two.y);
      return f;
    }
    
    public void mapconv(){
      for(Location l : impasse){
        optimized_impasse.put(hashCode(l.x,l.y),true);
      }
    }
    

    public IDA(ArrayList<Location> impasse, Location start, Location goal){
      this.impasse = impasse;
      this.optimized_impasse = new HashMap<Integer, Boolean>();
      this.start = start;
      this.goal = goal;
      this.parent_pointers = new HashMap<Integer,Integer>();
      this.BFS_Path = new ArrayList<Integer>();
    }
    //start off with heuristic value = 0
    public ArrayList<Location> IDA_Algo(Location start){
      return new ArrayList<Location>(); 
    }
    
    public ArrayList<Integer> backtrack(int end){
      ArrayList<Integer> res = new ArrayList<Integer>();
      while(true){
        res.add(end);
        if(!parent_pointers.containsKey(end)){break;}
        end= parent_pointers.get(end);
      }
      return res;
    }
    
    public ArrayList<Location> iterative_deepening_astar(){
      return new ArrayList<Location>();
    }
    
    public ArrayList<Integer> BFS(){
      Queue<Location> q = new LinkedList<Location>();
      q.add(this.start);
      float[] dx = {1,0,-1,0};
      float[] dy = {0,-1,0,1};
      ArrayList<Location> seen = new ArrayList<Location>();
      ArrayList<Integer> ret = new ArrayList<Integer>();
      while(q.size()>0){
        Location tp = q.poll();
        if(tp.x==this.goal.x&&tp.y==this.goal.y){break;}
        println("POINT: " + tp.x + " " + tp.y);
        for(int idx = 0; idx < dx.length; idx++){
          Location nxt = new Location(tp.x+dx[idx],tp.y+dy[idx]);
          if(parent_pointers.containsKey(nxt)){
            continue;
          }
          if(optimized_impasse.containsKey(nxt)){
            continue;
          }
            parent_pointers.put(hashCode(nxt.x,nxt.y),hashCode(tp.x,tp.y));
            println("ADD: " + nxt.x + " " + nxt.y);
            seen.add(nxt);
            q.add(nxt);
          }
       }
      ret = backtrack(hashCode(this.goal.x,this.goal.y));
      this.BFS_Path = ret;
      return ret;
    }
}
