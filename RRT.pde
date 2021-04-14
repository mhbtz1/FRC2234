import java.io.*;
import java.util.*;


class RRT{
   PVector seed;
   float dq;
   float MAX_ITER;
   float INTERNAL_COUNTER;
   HashMap<PVector, ArrayList<PVector> > graph;
   HashMap<PVector, Boolean> hset = new HashMap<PVector, Boolean>();
   ArrayList<PVector> seen_space = new ArrayList<PVector>();
   
   public boolean vec_contains(PVector v){
     if(hset.containsKey(v)){return true;}return false;
   }
   
   public RRT(PVector seed, float dq, float MAX_ITER){
     this.seed = seed;
     this.dq = dq;
     this.MAX_ITER = MAX_ITER;
     graph = new HashMap<PVector, ArrayList<PVector> >();
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
       PVector rp = new PVector( random(0,1000), random(0,1000) );
       PVector closest = nearest_point(rp);
       float ang = atan( (float)(closest.y-rp.y)/(float)(closest.x - rp.x) );
       PVector new_pt = new PVector(rp.x + (dq*cos(ang)), rp.y + (dq*sin(ang)) );
       if(graph.containsKey(closest)){
         ArrayList<PVector> tmp = graph.get(closest);
         tmp.add(new_pt);
         graph.put(closest,tmp);
         println(closest.x + " " + closest.y + " " + new_pt.x + " " + new_pt.y);
       } else {
         ArrayList<PVector> tmp = new ArrayList<PVector>();
         tmp.add(new_pt);
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
     ArrayList<PVector> see = graph.get(state);
     //for(PVector p : see){println(p.x + " " + p.y);}

     fill(0,0,255);
     while(qp.size() > 0){
       if(qp.size()==0){break;}
       PVector nxt = qp.poll();
       println("SIZE: " + qp.size());
       ArrayList<PVector> adj = new ArrayList<PVector>();
       if(graph.containsKey(nxt)){
          adj = graph.get(nxt);
       } else {
         continue;
       }
       println("CURRENT POSITION: " + nxt.x + " " + nxt.y);
       circle(nxt.x,nxt.y,10);
       for(PVector mvec : adj){
         if(!vec_contains(mvec)){
           println("ADDING POINT: " + mvec.x + " " + mvec.y);
           circle(mvec.x,mvec.y,10);
           println("LINE DRAWN");
           line(nxt.x,nxt.y,mvec.x,mvec.y);
           hset.put(mvec,true);
           qp.add(mvec);
         } else {
           println("ALREADY SEEN");
         }
       }
     }
     
     println("----------------------------------------------------------------------------");
    
   }
   
   public void reset(){
     hset.clear();
   }
}
