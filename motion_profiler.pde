//cubic bezier curve can be represented as:
//B(x) = (1-x)^3 * (P0) + 3x(1-x)^2 * (P1) + 3x^2(1-x)*(P2) + x^3 * (P3)

class Point{
  float x, y;
  public Point(float x, float y){
    this.x = x;
    this.y = y;
  }
}

//objective: create a smooth profile of cubic bezier curves with 4 control points that some point robot can follow a path along.
//let P(n,k) be the nth control point (n <= 3) for the kth bezier curve:

//for n=0 and n=3, it is trivial, these points are given to be one of the pre-existing waypoints.

//for n=2, P(2,i) = 2*K(i+1) - P(1,i+1) for 0 <= i <= n-2
// P(2,n-1) = 0.5*(K(n) + P(1,n-1))



void thomas_algorithm(){
  
}
float sigmoid_spline(float x){
  return (float)( ((20)/(1+pow((float)(Math.E),0.2*x)))* 20) + 300;
}


//cubic bezier profile
class BezierProfile{
  float init_x, init_y;
  float ref1_x, ref1_y;
  float ref2_x, ref2_y;
  float final_x, final_y;
  public BezierProfile(float init_x, float init_y, float ref1_x,float ref1_y, float ref2_x, float ref2_y, float final_x, float final_y){
    this.init_x = init_x;
    this.init_y = init_y;
    this.ref1_x = ref1_x;
    this.ref1_y = ref1_y;
    this.ref2_x = ref2_x;
    this.ref2_y = ref2_y;
    this.final_x = final_x;
    this.final_y = final_y;
  }
  
  //takes some input [0,1]
  float return_x(float x){
  double v = (init_x * Math.pow( (1-x), 3)) +(3*x * (1-x) * (1-x) * ref1_x) + (3*x*x * (1-x) * ref2_x)+ (Math.pow(x,3) * final_x);
  return (float)(v);
}
float return_y(float x){
   double v = (init_y * Math.pow( (1-x), 3)) +(3*x * (1-x)*(1-x) * ref1_y) + (3*x*x * (1-x) * ref2_y)+ (Math.pow(x,3) * final_y);
  return (float)(v);
}
}

class MotionProfiler {
  public ArrayList<BezierProfile> bez;
  public ArrayList<Point> waypoints;
  public ArrayList<Point> true_waypoints;
  public HashMap<Integer, PVector> velocity;
  public MotionProfiler(ArrayList<BezierProfile> bez, ArrayList<Point> waypoints){
    this.bez = bez;
    this.waypoints = waypoints;
    this.true_waypoints = new ArrayList<Point>();
  }
  public MotionProfiler(){
    this.bez = new ArrayList<BezierProfile>();
    this.waypoints = new ArrayList<Point>();
    this.true_waypoints = new ArrayList<Point>();
    this.velocity = new HashMap<Integer, PVector>();
  }
  
  public ArrayList<Point> parseFile(){
    ArrayList<Point> ans = new ArrayList<Point>();
    String[] r = loadStrings("controlPoints.txt");
    println("SIZE: "  + r.length);
    for(String line : r){
       double one = Double.parseDouble(line.substring(0, line.indexOf(":")));
       double two = Double.parseDouble(line.substring(line.indexOf(":")+1));
       ans.add(new Point( (float)(one), (float)(two) ) );
       println("POINT: " + one + " " + two);
    }
    return ans;
  }

  public void iterate_profiles(){
    if(waypoints.size() == 0){
      waypoints = parseFile();
      int ITER = 320;
      println("RUN");
      if(bez.size() == 0){
        for(int i = 0; i < waypoints.size()-ITER; i+= ITER){
          //println(waypoints.get(i).x + " " + waypoints.get(i).y);
          BezierProfile b = new BezierProfile(waypoints.get(i).x,waypoints.get(i).y,waypoints.get(i+ITER/4).x,waypoints.get(i+ITER/4).y,waypoints.get(i+ITER/2).x,waypoints.get(i+ITER/2).y,waypoints.get(i+ITER).x,waypoints.get(i+ITER).y);
          bez.add(b);
        }
      }
    }
    for(BezierProfile b : bez){
      for(float j = 0; j <= 1.0; j += 0.04){
        fill(255,0,0);
        println(b.return_x(j) + " " + b.return_y(j));
        true_waypoints.add(new Point(b.return_x(j), b.return_y(j)));
        fill(255,0,0);
        ellipse(b.return_x(j), b.return_y(j), 5, 5);
      }
    }
  }
  
  public void generate_primitive_profiles(){
    for(int i = 0; i < true_waypoints.size(); i++){
    }
  }
  
  
  
}

 
