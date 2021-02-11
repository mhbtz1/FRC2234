package frc.robot;

public class PIDControl {
    double TOT_ERROR = 0;
    double INC_ERROR =0;
    double P, I, D;
    public PIDControl(double P, double I, double D){
        this.P= P;
        this.I = I;
        this.D = D;
    }
    public void error_reset(){
        TOT_ERROR = 0;
        INC_ERROR = 0;
    }

}
