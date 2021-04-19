//iterative deepening A*
import java.util.*;



//the nice thing about using RRTs for making continuous stuff discrete is that the graph structure
//already encodes the obstacle space for us.



class IDAComparator implements Comparator<GType>{
  public int compare(GType one, GType two){
     if(one.COST_TO_REACH + one.HEURISTIC_COST < two.COST_TO_REACH + two.HEURISTIC_COST){
       return 1;
     } else {
       return 0;
     }
  }
}

class IDA{
    HashMap<PVector, Boolean> vis = new HashMap<PVector, Boolean>();
    HashMap<PVector, ArrayList<GType> > graph = new HashMap<PVector, ArrayList<GType> >();
    HashMap<PVector, Float> intermediate_results = new HashMap<PVector, Float>();
    //stores a list of integers which encode the positions they represent
    ArrayList<PVector> ASTAR_PATH;
    PVector start,goal;
    HashMap<PVector, PVector> parent_pointers = new HashMap<PVector, PVector>();
    public int hashCode(float x, float y){
      int hash = (int)( 1400*(y) + x );
      //print("HASH:" + hash); 
      return hash;
    }
    public float euclidean_heuristic_function(Location one, Location two){
      float f = dist(one.x,one.y,two.x,two.y);
      return f;
    }
    

    public IDA(HashMap<PVector, ArrayList<GType> > graph, Location start, Location goal){
      this.start = new PVector(start.x,start.y);
      this.goal = new PVector(goal.x,goal.y);
      this.parent_pointers = new HashMap<PVector, PVector>();
      this.ASTAR_PATH = new ArrayList<PVector>();
      this.graph = graph;
    }
    //start off with heuristic value = 0
    public ArrayList<Location> IDA_Algo(Location start){
      return new ArrayList<Location>(); 
    }
    
    
    public ArrayList<PVector> backtracking(PVector fin){
      ArrayList<PVector> res = new ArrayList<PVector>();
      while(fin != null){
        res.add(fin);
        fin = parent_pointers.get(fin);
      }
      return res;
    }
    /*
    public ArrayList<Integer> backtrack(int end){
      println("START BACKTRACKING");
      ArrayList<Integer> res = new ArrayList<Integer>();
      while(true){
        println("CURRENT POSITION: " + end);
        res.add(end);
        if(!parent_pointers.containsKey(end)){break;}
        end= parent_pointers.get(end);
      }
      return res;
    }
    */
    
    public ArrayList<Location> iterative_deepening_astar(){
      return new ArrayList<Location>();
    }
    
    /*
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
          if(parent_pointers.containsKey(hashCode(nxt.x,nxt.y))){
            continue;
          }
          if(optimized_impasse.containsKey(hashCode(nxt.x,nxt.y))){
            continue;
          }
            parent_pointers.put(hashCode(tp.x,nxt.y),hashCode(tp.x,tp.y));
            println("ADD: " + nxt.x + " " + nxt.y);
            seen.add(nxt);
            q.add(nxt);
          }
       }
      //ret = backtrack(hashCode(this.goal.x,this.goal.y));
      //this.BFS_Path = ret;
      return ret;
    }
    */
    
    public ArrayList<PVector> IDA(){
       PriorityQueue<GType> pq = new PriorityQueue(new IDAComparator());
       GType seed = new GType(start, 0, dist(start.x,start.y,goal.x,goal.y));
       pq.add(seed);
       ArrayList<PVector> path = new ArrayList<PVector>();
       while(pq.size() > 0){
         GType frnt = pq.poll();
         float f_cost = frnt.COST_TO_REACH;
         float g_cost = frnt.HEURISTIC_COST;
         ArrayList<GType> adj = graph.get(frnt.evec);
         for(GType p : adj){
           float new_f = f_cost + p.COST_TO_REACH;
           float new_g = dist(p.evec.x,p.evec.y,goal.x,goal.y);
           if(!intermediate_results.containsKey(p.evec)){
             intermediate_results.put(p.evec, new_f + new_g);
             GType g = new GType(p.evec, new_f, new_g);
             pq.add(g);
           } else {
             if(Math.min(intermediate_results.get(p.evec), new_f + new_g) == new_f + new_g){
               GType ng = new GType(p.evec, new_f, new_g);
               pq.add(ng);
               parent_pointers.put(p.evec, frnt.evec);
             }
           }
         }
       }
       
       return new ArrayList<PVector>();
    }
}
