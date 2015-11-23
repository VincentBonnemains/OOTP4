package utils;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.image.BufferedImage;

import javax.swing.JPanel;

import simbad.demo.DemoTp4;
import simbad.sim.Agent;

public class ImagerPanel extends JPanel {
	private static final long serialVersionUID = 1L;
	private Agent robot;
	
	public ImagerPanel(Agent robot) {
		this.robot = robot;
	}
	
/*	protected void paintComponent(Graphics g) {
        int width = ((DemoTp4.Robot)robot).bufferedMatrices[0].getWidth();
        int height = ((DemoTp4.Robot)robot).bufferedMatrices[0].getHeight();
        super.paintComponent(g);
        g.setColor(Color.WHITE);
        g.fillRect(0, 0, width, height);
        
        for(int i=0;i<((DemoTp4.Robot)robot).bufferedMatrices.length;i++){
            for (int y = 0; y < height; y += 1) {
                for (int x = 0; x < width; x += 1) {
                    int color = ((DemoTp4.Robot)robot).bufferedMatrices[i].getRGB(x, y);
                    //int alpha = (color >> 24) & 0xFF;
                    int red =   (color >> 16) & 0xFF;
                    int green = (color >>  8) & 0xFF;
                    int blue =  (color      ) & 0xFF;
                    //System.out.println("R:"+red+" G:"+green+" B:"+blue);
                    
                    Color c = new Color(red,green,blue);
                    g.setColor(c);
                    
                    g.fillRect(x+i*width, y, 1, 1);
                       
                }
            }
        }
    }*/
	
	protected void paintComponent(Graphics g) {
		BufferedImage bi = ((DemoTp4.Robot)robot).frontMatrix;
        int width = bi.getWidth();
        int height = bi.getHeight();
        super.paintComponent(g);
        g.setColor(Color.WHITE);
        g.fillRect(0, 0, width, height);
        
            for (int y = 0; y < height; y += 1) {
                for (int x = 0; x < width; x += 1) {
                    int color = bi.getRGB(x, y);
                    //int alpha = (color >> 24) & 0xFF;
                    int red =   (color >> 16) & 0xFF;
                    int green = (color >>  8) & 0xFF;
                    int blue =  (color      ) & 0xFF;
                    //System.out.println("R:"+red+" G:"+green+" B:"+blue);
                    
                    Color c = new Color(red,green,blue);
                    g.setColor(c);
                    
                    g.fillRect(x, y, 1, 1);
                       
                }
            }
        
    }
}
