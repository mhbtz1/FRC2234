//cubic bezier profile
package frc.robot;
import java.util.*;

class BezierProfile{
        double init_x, init_y;
        double ref1_x, ref1_y;
        double ref2_x, ref2_y;
        double final_x, final_y;
        

	public BezierProfile(double x, double y, double x2, double y2, double x3, double y3, double x4, double y4) {
            this.init_x = x;
            this.init_y = y;
            this.ref1_x = x2;
            this.ref1_y = y2;
            this.ref2_x = x3;
            this.ref2_y = y3;
            this.final_x = x4;
            this.final_y = y4;
		}
        double return_x(double x){
            double v = (init_x * Math.pow( (1-x), 3)) +(3*x * (1-x) * (1-x) * ref1_x) + (3*x*x * (1-x) * ref2_x)+ (Math.pow(x,3) * final_x);
            return v;
        }
        double return_y(double x){
            double v = (init_y * Math.pow( (1-x), 3)) +(3*x * (1-x)*(1-x) * ref1_y) + (3*x*x * (1-x) * ref2_y)+ (Math.pow(x,3) * final_y);
            return v;
        }
  }
