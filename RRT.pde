import java.io.*;
import java.util.*;


//implement KD Tree for fast closest point queries

//also add some stuff for finding the voronoi regions of our current RRT state, and sample on the sizes
//of the RRT's voronoi regions


class KDTree{
  public KDTree(){
  }
}




class RRT{
   PVector seed;
   float dq;
   float MAX_ITER;
   float INTERNAL_COUNTER;
   HashMap<PVector, ArrayList<GType> > graph;
   HashMap<PVector, Boolean> hset = new HashMap<PVector, Boolean>();
   ArrayList<PVector> seen_space = new ArrayList<PVector>();
   ArrayList<Location> obstacle_space= new ArrayList<Location>();
   //we want to use some dispersion heuristic, so 
   
   public boolean vec_contains(PVector v){
     if(hset.containsKey(v)){return true;}return false;
   }
   
   public RRT(PVector seed, float dq, float MAX_ITER){
     this.seed = seed;
     this.dq = dq;
     this.MAX_ITER = MAX_ITER;
     graph = new HashMap<PVector, ArrayList<GType> >();
     seen_space.add(seed);
     this.INTERNAL_COUNTER = 0;
   }
   //this method is kind of computationally heavy
   public PVector nearest_point(PVector comp){
     float mdist  = 1000000000;
     PVector cur = new PVector(0,0);
     for(PVector v : seen_space){
       mdist = min(mdist, dist(comp.x,comp.y,v.x,v.y));
       if(mdist == dist(comp.x,comp.y,v.x,v.y)){
         mdist = dist(comp.x,comp.y,v.x,v.y);
         cur = new PVector(v.x,v.y);
       }
     }
     return cur;
   }
   
   public boolean rrtExploration(){
     if(this.INTERNAL_COUNTER <= this.MAX_ITER){
       println("INTERNAL COUNTER: " + this.INTERNAL_COUNTER);
       PVector rp = new PVector( random(0,1400), random(0,900) );
       float seed = random(0,1);
       
       PVector closest = nearest_point(rp);
       float ang = atan( (float)(rp.y-closest.y)/(float)(rp.x-closest.x) );
       PVector new_pt = new PVector(closest.x + (dq*cos(ang)), closest.y + (dq*sin(ang)) );
       if(graph.containsKey(closest)){
         ArrayList<GType> tmp = graph.get(closest);
         tmp.add( new GType(new_pt, dist(closest.x,closest.y,new_pt.x,new_pt.y) ));
         graph.put(closest,tmp);
         println(closest.x + " " + closest.y + " " + new_pt.x + " " + new_pt.y);
       } else {
         ArrayList<GType> tmp = new ArrayList<GType>();
         tmp.add( new GType(new_pt, dist(closest.x,closest.y,new_pt.x,new_pt.y)) );
         println(closest.x + " " + closest.y + " " + new_pt.x + " " + new_pt.y);
         graph.put(closest, tmp);
       }
       seen_space.add(new_pt);
       this.INTERNAL_COUNTER++;
       return true;
     }
     return false;
   }
   
   public void displayRRT(PVector state){
     println("----------------------------------------------------------");
     LinkedList<PVector> qp = new LinkedList<PVector>();
     qp.add(state);
     //for(PVector p : seen_space){fill(255,0,0); circle(p.x,p.y,10);}
     ArrayList<GType> see = graph.get(state);
     //for(PVector p : see){println(p.x + " " + p.y);}
     circle(state.x,state.y,10);
     stroke(0,0,255);
     fill(0,0,255);
     while(qp.size() > 0){
       if(qp.size()==0){break;}
       PVector nxt = qp.poll();
       println("SIZE: " + qp.size());
       ArrayList<GType> adj = new ArrayList<GType>();
       if(graph.containsKey(nxt)){
          adj = graph.get(nxt);
       } else {
         continue;
       }
       println("CURRENT POSITION: " + nxt.x + " " + nxt.y);
       for(GType mvec : adj){
         if(!vec_contains(mvec.evec)){
           println("ADDING POINT: " + mvec.evec.x + " " + mvec.evec.y);
           circle(mvec.evec.x,mvec.evec.y,10);
           println("LINE DRAWN: " + nxt.x + " " + nxt.y + " " + mvec.evec.x +  " " + mvec.evec.y);
           line(nxt.x,nxt.y,mvec.evec.x,mvec.evec.y);
           hset.put(mvec.evec,true);
           qp.add(mvec.evec);
         } else {
           println("ALREADY SEEN");
         }
       }
     }
     println("----------------------------------------------------------------------------");
    
   }
   
   public boolean detectObstacle(PVector one, PVector two){
     return true;
   }
   
   public void reset(){
     hset.clear();
   }
}
