import java.io.*;
import java.util.*;


class RRT{
   PVector seed;
   float dq;
   float maxIter;
   ArrayList[] adj_list = new ArrayList[100001];
   ArrayList<PVector> seen_space = new ArrayList<PVector>();
   public RRT(PVector seed, float dq, float maxIter){
     this.seed = seed;
     this.dq = dq;
     this.maxIter = maxIter;
     for(int i = 0; i < 100001; i++){
       adj_list[i] = new ArrayList<Integer>();
     }
     seen_space.add(seed);
   }
   public PVector nearest_point(){
     PVector cur = new PVector(0,0);
     float mdist  = 1000000000;
     for(PVector v : seen_space){
       mdist = min(mdist, dist(cur.x,cur.y,v.x,v.y));
       if(mdist == dist(cur.x,cur.y,v.x,v.y)
     }
   }
   
   public void rrtExploration(){
     
     PVector rp = new PVector( random(0,1000), random(0,1000) );
     PVector closest = nearest_point();
     float ang = atan( (float)(closest.y-rp.y)/(float)(closest.x - rp.x) );
     PVector new_pt = 
   }
}
