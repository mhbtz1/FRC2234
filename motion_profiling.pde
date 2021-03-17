//cubic bezier curve can be represented as:
//B(x) = (1-x)^3 * (P0) + 3x(1-x)^2 * (P1) + 3x^2(1-x)*(P2) + x^3 * (P3)

class Point{
  float x, y;
  public Point(float x, float y){
    this.x = x;
    this.y = y;
  }
}

ArrayList<BezierProfile> bez = new ArrayList<BezierProfile>();
ArrayList<Point> waypoints = new ArrayList<Point>();


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
void gen_waypoints(){
  for(int i = -400; i < 1000; i += 100){
        waypoints.add(new Point( i+500, sigmoid_spline(i) ) );
  }
}


void setup(){
  size(1400,1000);
  for(int i = 0; i < 20; i++){
    bez.add(new BezierProfile(800,500,200,100,300,300,400,400));
  }
  gen_waypoints();
  fill(255,0,0);
  for(Point p : waypoints){
    ellipse(p.x,p.y,10,10);
  }
  

}

void draw(){

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
  float return_x(float x){
  double v = (init_x * Math.pow( (1-x), 3)) +(3*x * (1-x) * (1-x) * ref1_x) + (3*x*x * (1-x) * ref2_x)+ (Math.pow(x,3) * final_x);
  return (float)(v);
}
float return_y(float x){
   double v = (init_y * Math.pow( (1-x), 3)) +(3*x * (1-x)*(1-x) * ref1_y) + (3*x*x * (1-x) * ref2_y)+ (Math.pow(x,3) * final_y);
  return (float)(v);
}



  
}
