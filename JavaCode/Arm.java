 


/**
 * Class represents SCARA robotic arm.
 * 
 * @Arthur Roberts
 * @0.0
 */

import ecs100.UI;
import java.awt.Color;
import java.util.*;

public class Arm
{
    
    // fixed arm parameters
    private int xm1;  // coordinates of the motor(measured in pixels of the picture)
    private int ym1;
    private int xm2;
    private int ym2;
    private double r;  // length of the upper/fore arm
    // parameters of servo motors - linear function pwm(angle)
    // each of two motors has unique function which should be measured
    // linear function cam be described by two points
    // motor 1, point1 
    private double pwm1_val_1; 
    private double theta1_val_1;
    // motor 1, point 2
    private double pwm1_val_2; 
    private double theta1_val_2;
    
    // motor 2, point 1
    private double pwm2_val_1; 
    private double theta2_val_1;
    // motor 2, point 2
    private double pwm2_val_2; 
    private double theta2_val_2;
    
    
    // current state of the arm
    private double theta1; // angle of the upper arm
    private double theta2;
    
    private int motor1;
    private int motor2;
    
    private double xj1;     // positions of the joints
    private double yj1; 
    private double xj2;
    private double yj2; 
    private double xt;     // position of the tool
    private double yt;
    public boolean valid_state; // is state of the arm physically possible?
    
    /** Angles to PWM formula constants */
    static final private double m1 = 0.0968;
    static final private double c1 = 24.986;
    static final private double m2 = 0.0906;
    static final private double c2 = 73.035;
    
    /**
     * Constructor for objects of class Arm
     */
    public Arm()
    {
        xm1 = 290; // set motor coordinates
        ym1 = 372;
        xm2 = 379;
        ym2 = 374;
        r = 156.0;
        theta1 = -90.0*Math.PI/180.0; // initial angles of the upper arms
        theta2 = -90.0*Math.PI/180.0;
        valid_state = false;
    }
  
    // draws arm on the canvas
    public void draw()
    {
        // draw arm
        int height = UI.getCanvasHeight();
        int width = UI.getCanvasWidth();
        // calculate joint positions
        xj1 = xm1 + r*Math.cos(theta1);
        yj1 = ym1 + r*Math.sin(theta1);
        xj2 = xm2 + r*Math.cos(theta2);
        yj2 = ym2 + r*Math.sin(theta2);
        
        //draw motors and write angles
        int mr = 20;
        UI.setLineWidth(1);
        UI.setColor(Color.BLUE);
        UI.drawOval(xm1-mr/2,ym1-mr/2,mr,mr);
        UI.drawOval(xm2-mr/2,ym2-mr/2,mr,mr);
        // write parameters of first motor
        String out_str=String.format("t1=%3.1f",theta1*180/Math.PI);
        UI.drawString(out_str, xm1-2*mr,ym1-mr/2+2*mr);
        out_str=String.format("xm1=%d",xm1);
        UI.drawString(out_str, xm1-2*mr,ym1-mr/2+3*mr);
        out_str=String.format("ym1=%d",ym1);
        UI.drawString(out_str, xm1-2*mr,ym1-mr/2+4*mr);
        out_str=String.format("motor1=%d",motor1);
        UI.drawString(out_str, xm1-2*mr,ym1-mr/2+5*mr);
        
        // ditto for second motor                
        out_str = String.format("t2=%3.1f",theta2*180/Math.PI);
        UI.drawString(out_str, xm2+2*mr,ym2-mr/2+2*mr);
        out_str=String.format("xm2=%d",xm2);
        UI.drawString(out_str, xm2+2*mr,ym2-mr/2+3*mr);
        out_str=String.format("ym2=%d",ym2);
        UI.drawString(out_str, xm2+2*mr,ym2-mr/2+4*mr);
        out_str=String.format("motor2=%d",motor2);
        UI.drawString(out_str, xm2+2*mr,ym2-mr/2+5*mr);
        // draw Field Of View
        UI.setColor(Color.GRAY);
        UI.drawRect(0,0,640,480);
         
       // it can b euncommented later when
       // kinematic equations are derived
        if ( valid_state) {
          // draw upper arms
          UI.setColor(Color.GREEN);
          UI.drawLine(xm1,ym1,xj1,yj1);
          UI.drawLine(xm2,ym2,xj2,yj2);
          //draw forearms
          UI.drawLine(xj1,yj1,xt,yt);
          UI.drawLine(xj2,yj2,xt,yt);
          // draw tool
          double rt = 20;
          UI.drawOval(xt-rt/2,yt-rt/2,rt,rt);
        }
        
   }
    
   // calculate tool position from motor angles 
   // updates variable in the class
   public void directKinematic(){
       
       // midpoint between joints
       //double  xa =.... ;
       //double  ya =.... ;
       // distance between joints
       //double d = ...;
       //if (d<2*r){
           valid_state = true;
         // half distance between tool positions
         //double  h = ...;
         //double alpha= ...;
         // tool position
        // double xt = ...;
        // double yt = ...;
         //  xt2 = xa - h.*cos(alpha-pi/2);
         //  yt2 = ya - h.*sin(alpha-pi/2);
       //} else {
        //   valid_state = false;
        //}
       
    }
    
    // motor angles from tool position
    // updetes variables of the class
    public void inverseKinematic(double xt_new,double yt_new){
        /** singularities
           -motor1 too far left
           -motor2 too far right
           -angle at joint not too small so pen doesnt go below
           the join
           */
        valid_state = true;
        xt = xt_new;
        yt = yt_new;
        valid_state = true;
        
        /*# First motor */
        
        double dx1 = xt - xm1; 
        double dy1 = yt - ym1;
        // distance between pem and motor
        double d1 = Math.sqrt(dx1*dx1+dy1*dy1);
        if (d1>2*r){
            UI.println("Arm 1 - can not reach");
            valid_state = false;
            return;
        }
        
        
        double l1 = d1/2;
        double h1 = Math.sqrt(r*r - l1*l1/4);
        
        double xa = xm1+0.5*(xt-xm1);
        double ya = ym1 +0.5*(yt-ym1);
        
        double alpha = Math.PI/2 - Math.atan2(yt-ym1, xm1-xt);
        
        // elbows positions
        xj1 = xa+h1*Math.cos(alpha);
        yj1 = ya+h1*Math.sin(alpha);
        
        double xj12 = xa-h1*Math.cos(alpha);
        double yj12 = ya-h1*Math.sin(alpha);

        theta1 = Math.PI-Math.atan2(yj1-ym1, xm1-xj1);       

        if (theta1<Math.PI || theta1>Math.PI*3/2){
            valid_state = false;
            UI.println("Angle 1 -invalid ");
            return;
       }
        
        double theta12 = Math.atan2(yj12 - ym1,xj12-xm1);
        
        /*# Second motor */
        
        double dx2 = xt - xm2; 
        double dy2 = yt - ym2;
        double d2 = Math.sqrt(dx2*dx2+dy2*dy2);
        if (d2>2*r){
            UI.println("Arm 2 - can not reach");
            valid_state = false;
            return;
        }

        double l2 = d2/2;
        double h2 = Math.sqrt(r*r - l2*l2/4);
        
        double xa2 = xm2 + 0.5 * (xm2-xt);
        double ya2 = ym2 + 0.5 * (ym2-yt);
        
        double alpha2 = Math.PI/2 - Math.atan2(ym2-yt, xt-xm2);
        
        // elbows positions
        xj2 = xa2-h2*Math.cos(alpha2);
        yj2 = ya2-h2*Math.sin(alpha2);
        
        double xj22 = xa2+h2*Math.cos(alpha2);
        double yj22 = ya2+h2*Math.sin(alpha2);
        
        // motor angles for both 1st elbow positions
        theta2 = Math.PI-Math.atan2(ym2-yj2, xj2-xm2);
        
        if ((theta2>2*Math.PI)||(theta2<Math.PI*3/2)){
            valid_state = false;
            UI.println("Angle 2 -invalid");
            return;
        }
        
        if((yj1+xj2)/2 <= yt){
            valid_state = false;
            UI.println("Pen too low");
            return;
        }
        
        double theta22 = Math.atan2(ym2-yj22,xm2-xj22);
        
        calculateMotorVoltages();
        
        UI.printf("xt:%3.1f, yt:%3.1f\n",xt,yt);
        UI.printf("theta1:%3.1f, theta2:%3.1f\n",360-theta1*180/Math.PI,360-theta2*180/Math.PI);
        return;
    }
    
    
    /** Calculates motor voltages based on angles */
    public void calculateMotorVoltages(){
        motor1 = (int) ((360-theta1*180/Math.PI+c1)/m1);
        motor2 = (int) ((360-theta2*180/Math.PI+c2)/m2);
    }
    
    // returns angle of motor 1
    public double get_theta1(){
        return theta1;
    }
    // returns angle of motor 2
    public double get_theta2(){
        return theta2;
    }
    // sets angle of the motors
    public void set_angles(double t1, double t2){
        theta1 = t1;
        theta2 = t2;
    }
    
    // returns motor control signal
    // for motor to be in position(angle) theta1
    // linear intepolation
    public int get_pwm1(){
        int pwm = (int) ((360-theta1*180/Math.PI+c1)/m1);
        return pwm;
    }
    // ditto for motor 2
    public int get_pwm2(){
        int pwm =(int) ((360-theta2*180/Math.PI+c2)/m2);
        return pwm;
    }
    
 }
