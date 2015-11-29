package utils;

import java.awt.image.BufferedImage;

import simbad.demo.Demo;
import simbad.demo.DemoTp4;
import simbad.sim.Agent;

public class RobotUtils {
	double lastError,error,integral,derivative,big_angle,kp, ki, kd, kb, hmin;
	Agent robot;
	double[] POURCENTAGE_COULEUR_FRONT,POURCENTAGE_COULEUR_BACK;
	public static int NOIR=0,VERT=1,BLANC=2,JAUNE=3,ROUGE = 4, BLEU = 5,GRIS=6;
	
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
		POURCENTAGE_COULEUR_FRONT = new double[2];
		POURCENTAGE_COULEUR_BACK  = new double[2];
		this.robot = robot;
	}
	
	public double pid() {
		error = POURCENTAGE_COULEUR_FRONT[0] - POURCENTAGE_COULEUR_FRONT[1];
    	integral = integral + error;
    	derivative = error - lastError;
    	lastError = error;
    	
    	if(Math.abs(hmin) < 90 && Math.abs(hmin) > 0) {
    		big_angle = ((Math.PI/40)/100) * hmin;
    	} else {
    		big_angle = 0;
    	}
    	
    	return kp*error + ki*integral + kd*derivative + kb*big_angle;
	}
	
	
	
	public boolean[] colorsDetectedFront(BufferedImage matrix) {
    	int w = matrix.getWidth();
    	int h = matrix.getHeight();
    	boolean[] couleurs = new boolean[((DemoTp4.Robot)robot).nbColors];
    	
    	POURCENTAGE_COULEUR_FRONT[0] = 0;
    	POURCENTAGE_COULEUR_FRONT[1] = 0;
		hmin = -1;    	
    	
    	for(int y=0;y<h;y++){
			for(int x=0;x<w;x++){
				int color = matrix.getRGB(x, y);
                int red =   (color >> 16) & 0xFF;
                int green = (color >>  8) & 0xFF;
                int blue =  (color      ) & 0xFF;
                if(red > 230 && green > 230 && blue < 120){ //Jaune détecté
                	couleurs[JAUNE] = true;
                } else if(red < 100 && green > 200 && blue < 200){ //Vert détecté
                	couleurs[VERT] = true;
                } else if(red > 100 && green <100 && blue < 100){ //Rouge détecté
                	couleurs[ROUGE] = true;
                } else if(red < 100 && green < 100 && blue > 100){ //Bleu détecté
                	couleurs[BLEU] = true;
                } else if(red < 50 && green < 50 && blue < 50){ //Noir détecté
                	couleurs[NOIR] = true;
            		if(x < (w/2)){
            			POURCENTAGE_COULEUR_FRONT[0]++;
            		}else{
            			POURCENTAGE_COULEUR_FRONT[1]++;
            		}
            		if(x==(w-1) && y > Math.abs(hmin)) {
            			hmin = -y;
            		}
            		else if(x==0 && y > Math.abs(hmin)){
            			hmin = y;          		
            		}                	
                } else if(red > 200 && green > 200 && blue > 200){ //BLANC détecté
                	couleurs[BLANC] = true;
                }
    		}
		}
    	POURCENTAGE_COULEUR_FRONT[0] /= w*h;
    	POURCENTAGE_COULEUR_FRONT[1] /= w*h;
		return couleurs;
    }
	
	public boolean[] colorsDetectedBack(BufferedImage matrix) {
    	int w = matrix.getWidth();
    	int h = matrix.getHeight();
    	boolean[] couleurs = new boolean[((DemoTp4.Robot)robot).nbColors];

    	
    	for(int y=0;y<h;y++){
			for(int x=0;x<w;x++){
				int color = matrix.getRGB(x, y);
                int red =   (color >> 16) & 0xFF;
                int green = (color >>  8) & 0xFF;
                int blue =  (color      ) & 0xFF;
                
                if(red > 140 && red < 160 && green > 140 && green < 160 && blue > 140 && blue < 160) { //Boite grise
                	couleurs[GRIS] = true;
                } else if(red > 230 && green > 230 && blue < 120){ //Jaune détecté
                	couleurs[JAUNE] = true;
                } else if(red < 100 && green > 200 && blue < 200){ //Vert détecté
                	couleurs[VERT] = true;
                } else if(red > 100 && green <100 && blue < 100){ //Rouge détecté
                	couleurs[ROUGE] = true;
                } else if(red < 100 && green < 100 && blue > 100){ //Bleu détecté
                	couleurs[BLEU] = true;
                } else if(red < 50 && green < 50 && blue < 50){ //Noir détecté
                	couleurs[NOIR] = true;            		        	
                } else if(red > 200 && green > 200 && blue > 200){ //BLANC détecté
                	couleurs[BLANC] = true;
                }
    		}
		}

		return couleurs;
    }  
}
