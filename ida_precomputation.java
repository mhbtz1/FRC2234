public static LinkedList<NodeObj> adj[];
public static int[] astar_predecessor;
public static PriorityQueue<NodeObj> pq_two;
public static HashMap<Integer, Location> animation_procedure;
public static HashMap<Integer, Double> intermediate_sp_heuristic;

class PQCMP implements Comparator<GType>{
  public int compare(GType one, GType two){
    if(one.COST_TO_REACH + one.HEURISTIC_DIST < two.COST_TO_REACH + two.HEURISTIC_DIST){
      return 1;
    }
    return 0;
 }
}


 public static boolean is_contained(ArrayList<NodeObj> a, NodeObj comp){
        for(NodeObj n : a){
            if(n.cost == comp.cost && n.cost_goal == comp.cost_goal && n.nd == comp.nd){
                return true;
            }
        }
        return false;
  }

class NodeObj{
    int nd;
    double cost;
    double cost_goal;
    public NodeObj(int nd, double cost, double cost_goal){
        this.nd=nd;
        this.cost=cost;
        this.cost_goal=cost_goal;
    }
    public NodeObj(int nd, double cost){
        this.nd=nd;
        this.cost=cost;
    }
}

class IDA{
  HashMap<PVector, ArrayList<GType> > p;
  HashMap<GType,Boolean> vis;
  PVector current_loc, goal_loc;
  HashMap<PVector,Float> SPDS;//short for shortest path data structure
  HashMap<PVector,PVector> parent_pointers;
  
  public IDA(HashMap<PVector, ArrayList<GType> > p, PVector current_loc, PVector goal_loc){
    this.p = p;
    this.current_loc = current_loc;
    this.goal_loc = goal_loc;
    vis = new HashMap<GType,Boolean>();
    SPDS = new HashMap<PVector, Float>();
    parent_pointers = new HashMap<PVector,PVector>();
  }
  
  void ida(){
    PriorityQueue<GType> pq = new PriorityQueue<GType>(new PQCMP());
    pq.add(new GType(current_loc, 0, dist(current_loc.x,current_loc.y,goal_loc.x,goal_loc.y)));
    while(pq.size()>0){
      GType top = pq.poll();
      ArrayList<GType> incident = p.get(top);
      if(incident.size()==0){continue;}
      for(int i = 0; i < incident.size(); i++){
        if(vis.containsKey(incident.get(i).evec)){
          continue;
        }
        float n_dist = top.COST_TO_REACH + incident.get(i).COST_TO_REACH;
        float s_dist = dist(incident.get(i).evec.x,incident.get(i).evec.y,goal_loc.x,goal_loc.y);
        if(!SPDS.containsKey(incident.get(i).evec)){
            parent_pointers.put(incident.get(i).evec,top.evec);
            SPDS.put(incident.get(i).evec, n_dist+s_dist);
            pq.add(new GType(incident.get(i).evec, n_dist,s_dist));
        } else {
          float tmp = SPDS.get(incident.get(i).evec);
          if(Math.min(tmp, n_dist+s_dist)==n_dist+s_dist){
            parent_pointers.put(incident.get(i).evec,top.evec);
            SPDS.put(incident.get(i).evec, n_dist+s_dist);
            pq.add(new GType(incident.get(i).evec, n_dist,s_dist));
          }
        }
      }
    }
  }
  
  
  void astar(NodeObj start_node, NodeObj goal_node){
          pq_two.add(start_node);
          HashMap<NodeObj,Boolean> seen = new HashMap<NodeObj,Boolean>();
          ArrayList<NodeObj> seen_two = new ArrayList<NodeObj>();
          int amt = 0;
          while(pq_two.size() > 0){
              ++amt;
              NodeObj tmp = pq_two.poll();
              if(!is_contained(seen_two,tmp)){
                  seen_two.add(tmp);
              } else {
                  continue;
              }
  
              LinkedList<NodeObj> adj_list = adj[tmp.nd];
              double relax_v = 100000000;
              for(int i = 0; i < adj_list.size(); i++){
                  NodeObj tp = adj_list.get(i);
                  double wt = tp.cost;
                  NodeObj addend = new NodeObj(tp.nd, tmp.cost + wt, tp.cost_goal);
                  relax_v = Math.min(relax_v, tmp.cost + wt);
                  double  cst = tmp.cost + wt + tmp.cost_goal;
                  if(!intermediate_sp_heuristic.containsKey(tp.nd)) {
                      intermediate_sp_heuristic.put(tp.nd, tmp.cost + wt + tmp.cost_goal);
                      astar_predecessor[tp.nd] = tmp.nd;
                      pq_two.add(addend);
                  } else {
                      intermediate_sp_heuristic.put(tp.nd, Math.min(intermediate_sp_heuristic.get(tp.nd), cst));
                      if(Math.min(intermediate_sp_heuristic.get(tp.nd), cst)==cst){
                          astar_predecessor[tp.nd] = tmp.nd;
                          pq_two.add(addend);
                      }
                  }
              }
          }
          System.out.println("AMOUNT: " + amt);
          System.out.println("OPTIMAL PATH LENGTH: " + intermediate_sp_heuristic.get(goal_node.nd));
      }
}
