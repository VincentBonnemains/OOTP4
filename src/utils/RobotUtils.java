package utils;

import java.awt.image.BufferedImage;

import simbad.demo.Demo;
import simbad.demo.DemoTp4;
import simbad.sim.Agent;

public class RobotUtils {
	double lastError,error,integral,derivative,big_angle,kp, ki, kd, kb, hmin;
	Agent robot;
	double[] POURCENTAGE_COULEUR;
	public static int NOIR=0,VERT=1,BLANC=2,JAUNE=3;
	
	public RobotUtils(Agent robot){
		lastError = 0;
		error = 0;
		integral = 0;
		derivative = 0;
		big_angle = 0;
		kp = 1 ;
		ki = 0.01;
		kd=10;
		kb = 35*((DemoTp4.Robot)robot).VMAX;
		hmin = -1;
		POURCENTAGE_COULEUR = new double[2];
		this.robot = robot;
	}
	
	public double pid() {
		error = POURCENTAGE_COULEUR[0] - POURCENTAGE_COULEUR[1];
    	integral = integral + error;
    	derivative = error - lastError;
    	lastError = error;
    	
    	if(Math.abs(hmin) < 60 && Math.abs(hmin) > 0) {
    		big_angle = ((Math.PI/16)/100) * hmin;
    	} else {
    		big_angle = 0;
    	}
    	
    	return kp*error + ki*integral + kd*derivative + kb*big_angle;
	}
	
	public boolean[] colorDetectedBySensor(int i) {
    	BufferedImage sensor = ((DemoTp4.Robot)robot).bufferedMatrices[i];
    	int w = sensor.getWidth();
    	int h = sensor.getHeight();
    	boolean[] couleurs = new boolean[4];
    	
    	
    	if(i==1) {
    		POURCENTAGE_COULEUR[0] = 0;
    		POURCENTAGE_COULEUR[1] = 0;
    		hmin = -1;
    	}
    	
    	for(int y=0;y<h;y++){
			for(int x=0;x<w;x++){
				int color = sensor.getRGB(x, y);
                int red =   (color >> 16) & 0xFF;
                int green = (color >>  8) & 0xFF;
                int blue =  (color      ) & 0xFF;
                
                if(red > 230 && green > 230 && blue < 120){ //Jaune détecté
                	couleurs[JAUNE] = true;
                } else if(red < 100 && green > 200 && blue < 200){ //Vert détecté
                	couleurs[VERT] = true;
                } else if(red < 50 && green < 50 && blue < 50){ //Noir détecté
                	couleurs[NOIR] = true;
                	if(i == 1) {
                		if(x < (w/2)){
                			POURCENTAGE_COULEUR[0]++;
                		}else{
                			POURCENTAGE_COULEUR[1]++;
                		}
                		if(x==(w-1)) {
                			if(y > Math.abs(hmin)){
                				hmin=-y;
                			}
                		}
                		if(x==0) {
                			if(y > Math.abs(hmin)){
                				hmin=y;
                			}                    		
                		}
                	}
                } else if(red > 200 && green > 200 && blue > 200){ //BLANC détecté
                	couleurs[BLANC] = true;
                }
    		}
		}
    	if(i == 1) {
			POURCENTAGE_COULEUR[0] /= w*h;
			POURCENTAGE_COULEUR[1] /= w*h;
    	}
		return couleurs;
    }   
}
