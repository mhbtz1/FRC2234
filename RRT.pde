import java.io.*;
import java.util.*;


class RRT{
   PVector seed;
   float dq;
   float MAX_ITER;
   float INTERNAL_COUNTER;
   HashMap<PVector, ArrayList<PVector> > graph;
   HashSet<PVector> hset = new HashSet<PVector>();
   ArrayList<PVector> seen_space = new ArrayList<PVector>();
   public RRT(PVector seed, float dq, float MAX_ITER){
     this.seed = seed;
     this.dq = dq;
     this.MAX_ITER = MAX_ITER;
     graph = new HashMap<PVector, ArrayList<PVector> >();
     seen_space.add(seed);
     this.INTERNAL_COUNTER = 0;
   }
   //this method is kind of computationally heavy
   public PVector nearest_point(){
     PVector cur = new PVector(0,0);
     float mdist  = 1000000000;
     for(PVector v : seen_space){
       mdist = min(mdist, dist(cur.x,cur.y,v.x,v.y));
       if(mdist == dist(cur.x,cur.y,v.x,v.y)){
         mdist = dist(cur.x,cur.y,v.x,v.y);
         cur = new PVector(v.x,v.y);
       }
     }
     return cur;
   }
   
   public boolean rrtExploration(){
     if(this.INTERNAL_COUNTER <= this.MAX_ITER){
       println("INTERNAL COUNTER: " + this.INTERNAL_COUNTER);
       PVector rp = new PVector( random(0,1000), random(0,1000) );
       PVector closest = nearest_point();
       float ang = atan( (float)(closest.y-rp.y)/(float)(closest.x - rp.x) );
       PVector new_pt = new PVector(rp.x + (dq*cos(ang)), rp.y + (dq*sin(ang)) );
       if(graph.containsKey(rp)){
         ArrayList<PVector> tmp = graph.get(rp);
         tmp.add(new_pt);
       } else {
         ArrayList<PVector> tmp = new ArrayList<PVector>();
         tmp.add(new_pt);
         graph.put(closest, tmp);
       }
       seen_space.add(new_pt);
       this.INTERNAL_COUNTER++;
       return true;
     }
     return false;
   }
   
   public void displayRRT(PVector state){
     Queue<PVector> qp = new LinkedList<PVector>();
     qp.add(state);
     while(qp.size() > 0){
       PVector nxt = qp.poll();
       ArrayList<PVector> adj = graph.get(nxt);
       circle(nxt.x,nxt.y,10);
       for(PVector mvec : adj){
         if(!hset.contains(mvec)){
           circle(mvec.x,mvec.y,10);
           line(nxt.x,nxt.y,mvec.x,mvec.y);
           hset.add(mvec);
           qp.add(mvec);
         }
       }
     }
   }
}
