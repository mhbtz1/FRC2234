package frc.robot;
import edu.wpi.first.wpilibj.*;
import com.revrobotics.*;
import java.util.*;

//technically a misnomer, im using bezier curves to approximate a better trapezoidal motion profiling
public class TrapezoidalProfile{
    Location start, end;
    Location inter0, inter1, inter2, inter3;
    ArrayList<Location> parametrized_speeds;
    BezierProfile b;
    double MAX_VEL;
    double TIME_DISP;
    double distance;
    double EPS = 0.01;
    double MULT = 1.2;
    public TrapezoidalProfile(Location ax, Location by, double TIME_DISP){
        this.start = ax;
        this.end= by;
        this.MAX_VEL = 1.5 * (this.distance/(double)this.TIME_DISP);
        this.inter0 = new Location(0,0);
        this.inter1 = new Location(this.TIME_DISP/(double)(3), this.MAX_VEL);
        this.inter2 = new Location(2 * this.TIME_DISP/(double)(3), this.MAX_VEL);
        this.inter3 = new Location(this.TIME_DISP, 0);
        this.distance = Location.distance(ax,by);
        this.parametrized_speeds = new ArrayList<Location>();
        this.TIME_DISP = TIME_DISP;
        this.b = new BezierProfile(inter0.x,inter0.y,inter1.x,inter1.y,inter2.x,inter2.y,inter3.x,inter3.y);
    }
    public void instantiate_speeds(){
        for(double i = 0; i <= 1.0; i += EPS){
            this.parametrized_speeds.add(new Location(this.b.return_x(i),this.b.return_y(i)));
            System.out.println("BEZIER VELOCITY: " + this.b.return_x(i) + " " + this.b.return_y(i));
        }
    }
}
