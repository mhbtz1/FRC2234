package frc.robot;
import edu.wpi.first.wpilibj.*;

public class Location{
    public double x, y;
    public Location(double x, double y){
        this.x=x;
        this.y=y;
    }
    public boolean equals(Location candidate){
        return candidate.x==x && candidate.y==y;
    }
    public static double distance(Location one, Location two){
        return (Math.ceil(Math.sqrt(Math.pow( (one.x-two.x),2) + Math.pow( (one.y-two.y),2))));
    }
    public Location add(Location addend){
        return new Location(this.x+addend.x,this.y+addend.y);
    }
    public Location subtract(Location subtractend){
        return new Location(this.y-subtractend.y, this.x - subtractend.x);
    }
    public static double magnitude(Location g){
        return Math.sqrt( (g.x * g.x) + (g.y * g.y) );
    }
    public static boolean is_greater_or_equal(Location one, Location two){
        return ( one.x >= two.x && one.y >= two.y);
    }
}
