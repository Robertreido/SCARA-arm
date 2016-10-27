
/* Code for Assignment ?? 
 * Name:
 * Usercode:
 * ID:
 */

import ecs100.*;
import java.util.*;
import java.io.*;
import java.awt.*;
import java.awt.Color;
import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;

/** <description of class Main>
 */
public class Main{

    private Arm arm;
    private Drawing drawing;
    private ToolPath tool_path;
    // state of the GUI
    private int state; // 0 - nothing
    // 1 - inverse point kinematics - point
    // 2 - enter path. Each click adds point  
    // 3 - enter path pause. Click does not add the point to the path

    /**      */
    public Main(){
        UI.initialise();
        UI.addButton("xy to angles", this::inverse);
        UI.addButton("Enter path XY", this::enter_path_xy);
        UI.addButton("Save path XY", this::save_xy);
        UI.addButton("Load path XY", this::load_xy);
        UI.addButton("Save path Ang", this::save_ang);
        UI.addButton("Load path Ang:Play", this::load_ang);
        UI.addButton("Create Square", this::createSquare);
        UI.addButton("Create Circle", this::createCircle);
        UI.addButton("Draw Snowman", this::drawSnowman);

        // UI.addButton("Quit", UI::quit);
        UI.setMouseMotionListener(this::doMouse);
        UI.setKeyListener(this::doKeys);

        //ServerSocket serverSocket = new ServerSocket(22); 
        this.arm = new Arm();
        this.drawing = new Drawing();
        this.tool_path = new ToolPath();
        this.run();
        arm.draw();
    }

    /**draws square*/
    public void createSquare(){
        double[] x = {280, 360, 360, 280, 280};
        double[] y = {107, 107, 187, 187, 107};
        state = 2;

        for(int i = 0; i< x.length; i++){
            doMouse("clicked", x[i], y[i]);
        }

    }

    /**draws circle */
    public void createCircle(){
        state = 2;
        double[] x = new double[51];
        double[] y1 = new double[51];
        double[] y2 = new double[51];
        for(int i =0; i < x.length; i++){
            x[i] = 270+(2*i);
            y1[i] = Math.sqrt(2500-(x[i]-320)*(x[i]-320))+147;
            doMouse("clicked", x[i], y1[i]);
        }
        for(int i = x.length-1; i >= 0; i--){
            y2[i] = -Math.sqrt(2500-(x[i]-320)*(x[i]-320))+147;
            doMouse("clicked", x[i], y2[i]);
        }
    }

    public void drawSnowman(){
        String fname = UIFileChooser.open("Image file to render");
        UI.println(fname);
        BufferedImage tempImage;
        try{
            tempImage = ImageIO.read(new File(fname));

            int rows = tempImage.getHeight();
            int cols = tempImage.getWidth();
            Boolean[][] image = new Boolean[cols][rows];
            for (int row = 0; row < rows; row++){
                for (int col = 0; col < cols; col++){                
                    Color c = new Color(tempImage.getRGB(col, row));
                    if(c.getBlue() > 125){
                        image[col][row] = false;  
                        //state = 3;
                        //doMouse("clicked", 270+col, 80+row);
                    } else {
                        image[col][row] = true;
                        state = 3;
                        doMouse("clicked", 270+col, 80+row);
                        state = 2;
                        doMouse("clicked", 270+col, 80+row);
                    }
                }

            }
            
            
            /*for (int row = 0; row < rows; row++){
                for (int col = 0; col < cols; col++){ 
                    if(image[col][row]){  
                        state = 3;
                        doMouse("clicked", 270+col, 80+row);
                        state = 2;
                        doMouse("clicked", 270+col, 80+row);
                    }
                }
            }*/
            
        } catch(Exception e){}

    }

    public void doKeys(String action){
        UI.printf("Key :%s \n", action);
        if (action.equals("b")) {
            // break - stop entering the lines
            state = 3;
            //

        }

    }

    public void doMouse(String action, double x, double y) {
        //UI.printf("Mouse Click:%s, state:%d  x:%3.1f  y:%3.1f\n",
        //   action,state,x,y);
        UI.clearGraphics();
        String out_str=String.format("%3.1f %3.1f",x,y);
        UI.drawString(out_str, x+10,y+10);
        // 
        if ((state == 1)&&(action.equals("clicked"))){
            // draw as 

            arm.inverseKinematic(x,y);
            arm.draw();
            return;
        }

        if ( ((state == 2)||(state == 3))&&action.equals("moved") ){
            // draw arm and path
            arm.inverseKinematic(x,y);
            arm.draw();

            // draw segment from last entered point to current mouse position
            if ((state == 2)&&(drawing.get_path_size()>0)){
                PointXY lp = new PointXY();
                lp = drawing.get_path_last_point();
                //if (lp.get_pen()){
                UI.setColor(Color.GRAY);
                UI.drawLine(lp.get_x(),lp.get_y(),x,y);
                // }
            }
            drawing.draw();
        }

        // add point
        if (   (state == 2) &&(action.equals("clicked"))){
            // add point(pen down) and draw
            UI.printf("Adding point x=%f y=%f\n",x,y);

            arm.inverseKinematic(x,y);

            if(arm.valid_state){
                drawing.add_point_to_path(x,y,true); // add point with pen down            
                arm.draw();
                drawing.draw();
                drawing.print_path();
            }
        }

        if (   (state == 3) &&(action.equals("clicked"))){
            // add point and draw
            //UI.printf("Adding point x=%f y=%f\n",x,y);
            drawing.add_point_to_path(x,y,false); // add point wit pen up

            arm.inverseKinematic(x,y);
            arm.draw();
            drawing.draw();
            drawing.print_path();
            state = 2;
        }

    }

    public void save_xy(){
        state = 0;
        String fname = UIFileChooser.save();
        drawing.save_path(fname);
    }

    public void enter_path_xy(){
        state = 2;
    }

    public void inverse(){
        state = 1;
        arm.draw();
    }

    public void load_xy(){
        state = 0;
        String fname = UIFileChooser.open();
        drawing.load_path(fname);
        drawing.draw();

        arm.draw();
    }

    // save angles into the file
    public void save_ang(){
        String fname = UIFileChooser.open();
        tool_path.convert_drawing_to_angles(drawing,arm,fname);
        tool_path.convert_angles_to_pwm(arm);
        tool_path.save_pwm_file(fname);
    }

    public void load_ang(){
    }

    public void run() {
        while(true) {
            arm.draw();
            UI.sleep(20);
        }
    }

    public static void main(String[] args){
        Main obj = new Main();
    }    

}
