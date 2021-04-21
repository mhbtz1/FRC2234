import java.io.*;
import java.util.*;

PVector COMPARE_POINT;
class PQComparator implements Comparator<PVector>{
  public int compare(PVector one, PVector two){
    if(dist(one.x,one.y,COMPARE_POINT.x,COMPARE_POINT.y)<=dist(two.x,two.y,COMPARE_POINT.x,COMPARE_POINT.y)){
      return 1;
    }
    return 0;
  }
}

class PQTwo implements Comparator<PVector>{
  public int compare(PVector one, PVector two){
    if(one.x > two.x){
      return 1;
    } else if(one.x < two.x){
      return -1;
    } else {
      if(one.y<two.y){
        return 1;
      } else if(one.y>two.y){
        return -1;
      } else {
        return 0;
      }
    }
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
   public static final int K = 10;
   //we want to use some dispersion heuristic, so 
   public ArrayList<Float> softmax(ArrayList<Float> vals){
      float sm = 0;
      ArrayList<Float> n = new ArrayList<Float>();
      for(int i = 0; i < vals.size(); i++){
         sm += Math.exp(vals.get(i));
      }
      for(int i = 0; i < vals.size(); i++){
         float nw = (float)(Math.exp(vals.get(i)))/(float)(sm);
         n.add(nw);
      }
      return n;
  }
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
   
   //returns next node to expand our RRT on some node
   //the idea is that the max of the max dists in the KNN should approximate the voronoi region areas (i.e. nodes that are far away from other nodes
   //should have a higher probability of being expanded)
   public int k_nearest_neighbors(int K){
     PriorityQueue<PVector> pq = new PriorityQueue(new PQComparator());
     ArrayList<Float> MAX_DISTS = new ArrayList<Float>();
     for(int i = 0; i < seen_space.size(); i++){
       COMPARE_POINT = seen_space.get(i);
       for(int j = 0; j < seen_space.size(); j++){
         if(i==j){continue;}
         pq.add(seen_space.get(j));
       }
       int idx = 0;
       float DIST = 1000000007;
       while(idx < K){
         PVector tp = pq.poll();
         DIST = min(DIST,dist(seen_space.get(i).x,seen_space.get(i).y,tp.x,tp.y));
         idx++;
       }
       MAX_DISTS.add( (float)(DIST)/(float)(K) );
       pq.clear();
     }
     ArrayList<Float> nxt = softmax(MAX_DISTS);
     ArrayList<PVector> sample = new ArrayList<PVector>();
     for(int i = 0; i < nxt.size(); i++){sample.add(new PVector(nxt.get(i),i));}
     Collections.sort(sample, new PQTwo());
     int idx = 0;
     float f = random(0,1);
     for(int i = 0; i < sample.size(); i++){
       if(sample.get(i).x <= f){
         idx = (int)sample.get(i).y;
         break;
       }
     }
     
     return idx;
   }
   
   
   public boolean rrtExploration(){
     println("INTERNAL COUNTER: " + this.INTERNAL_COUNTER);
       float seed = random(0,1);
       if(this.INTERNAL_COUNTER <= this.MAX_ITER){
         PVector rp = new PVector( random(0,1400), random(0,900) );
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
       } else if(this.INTERNAL_COUNTER > this.MAX_ITER/5 && this.INTERNAL_COUNTER < this.MAX_ITER){
         int nxt = k_nearest_neighbors(K);
         println("INDEX: " + nxt);//issue is we are choosing the same node to expand upon too many times
         PVector corres = seen_space.get(nxt);
         PVector rp = new PVector(random(0,1400),random(0,900));
         //PVector closest = nearest_point(rp);
         float ang = atan( (float)(rp.y-corres.y)/(float)(rp.x-corres.x) );
         PVector new_pt = new PVector(corres.x + (dq*(cos(ang))), corres.y + (dq*(sin(ang))) ) ;
         if(graph.containsKey(corres)){
           ArrayList<GType> tmp = graph.get(corres);
           tmp.add( new GType(new_pt, dist(corres.x,corres.y,new_pt.x,new_pt.y) ));
           graph.put(corres,tmp);
           //println(closest.x + " " + closest.y + " " + new_pt.x + " " + new_pt.y);
         } else {
           ArrayList<GType> tmp = new ArrayList<GType>();
           tmp.add( new GType(new_pt, dist(corres.x,corres.y,new_pt.x,new_pt.y)) );
           //println(closest.x + " " + closest.y + " " + new_pt.x + " " + new_pt.y);
           graph.put(corres, tmp);
         }
         seen_space.add(new_pt);
         this.INTERNAL_COUNTER++;
         return true;
      } else {
        return false;
      }
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
