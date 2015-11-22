/*
 *  Simbad - Robot Simulator
 *  Copyright (C) 2004 Louis Hugues
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, 
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 -----------------------------------------------------------------------------
 * $Author: sioulseuguh $ 
 * $Date: 2005/01/19 15:38:28 $
 * $Revision: 1.3 $
 * $Source: /cvsroot/simbad/src/simbad/demo/Demo.java,v $
 */
package simbad.demo;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;

import javax.swing.JInternalFrame;
import javax.swing.JPanel;
import javax.vecmath.Color3f;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import simbad.sim.Agent;
import simbad.sim.Box;
import simbad.sim.CameraSensor;

import simbad.sim.PastilleAgent;
import simbad.sim.Piece;
import simbad.sim.RangeSensorBelt;
import simbad.sim.RobotFactory;
import simbad.sim.Wall;
import utils.ImagerPanel;
import utils.RobotUtils;


/**
 * The Base class for all demos.
 */
public class DemoTp4 extends Demo {
	Piece[] petites_pieces = new Piece[3];
	Piece[] grandes_pieces = new Piece[3];
	public class Robot extends Agent {
		/*******************************************************
		 * ATTRIBUTES ******************************************
		 *******************************************************/
		private RobotUtils utils;
		final int GAUCHE=0,DROITE=1,NOIR=0,VERT=1,BLANC=2,JAUNE=3;
		int compteurPivotGauche=0,compteurPivotDroit=0;
		double elapsed;

		Point3d position,current_position;
		ArrayList<Piece> sac = new ArrayList<>();
        double tic = 0.02;

		double ANGLE = 0;
		
		public double VMAX = 1;
		double VMIN = 0.2;
		double VITESSE = VMAX;
		double compteurblanc = 0.0;
    	double coef = 8;
    	int sens = 1;
    	int nbColors = 5;
    	int nbCameraSensors = 3;
		ArrayList<boolean[]> COULEURS = new ArrayList<boolean[]>();
		public boolean retrouverChemin = false;
		
		CameraSensor[] sensors = new CameraSensor[nbCameraSensors];
		public BufferedImage[] bufferedMatrices = new BufferedImage[nbCameraSensors];
		RangeSensorBelt rangeSensor;
				
		JPanel panel;
        JInternalFrame window;
        
        /*******************************************************
         * END ATTRIBUTES **************************************
         *******************************************************/
        
        
        public Robot(Vector3d position, String name) {
            super(position, name);
            utils = new RobotUtils(this);
            sensors = RobotFactory.addCameraBeltSensor(this,sensors);
            rangeSensor = RobotFactory.addSonarBeltSensor(this);
            this.position = new Point3d();
            this.current_position = new Point3d();
            // prepare a buffer for storing image
            for(int i=0;i<bufferedMatrices.length;i++){
            	bufferedMatrices[i] = sensors[i].createCompatibleImage();
            }
            // Prepare UI panel for image display
            panel = new ImagerPanel(this);
            Dimension dim = new Dimension(bufferedMatrices[0].getWidth()*bufferedMatrices.length, bufferedMatrices[0].getHeight());
            panel.setPreferredSize(dim);
            panel.setMinimumSize(dim);
            setUIPanel(panel);
            
            //Machine a etat variable
            this.etat = REPOSTOCHEMIN;
    		this.ramassePetitouGrand = 0;
    		this.resteGrand = true;
    		this.restePetit = true;
    		this.aRamasse = false;
    		this.plein = false;
    		boolean t[] = new boolean[nbColors];
    		for(int i=0;i<nbColors;i++)
    			COULEURS.add(t);
        }         
        

        /** Initialize Agent's Behavior */
        public void initBehavior() {
        	elapsed = getLifeTime();
        	getCoords(current_position);
        	getCoords(position);
        	this.rotateY(-Math.PI/2);
        }

        /** Perform one step of Agent's Behavior */
        public void performBehavior() {
        	
            if ((getLifeTime() - elapsed) > tic) {
	            getCoords(current_position);
            	setTranslationalVelocity(VITESSE);
	            setRotationalVelocity(ANGLE);
	            machineAEtat();
            	
	            /*getCoords(current_position);
	            deplacerObjets();
	            getCoords(position); */           

	            /* Mettre à jour les capteurs */
            	for(int i=0;i<sensors.length;i++) {
                	sensors[i].copyVisionImage(bufferedMatrices[i]);
                	COULEURS.set(i, utils.colorDetectedBySensor(i));                	
                }
            	panel.repaint();
            	/* ***************************** */
            	getCoords(position);
            	elapsed = getLifeTime();
            }            
            
            if (collisionDetected()){
            	setTranslationalVelocity(0);
            }
        }
    	
        public void suiviDeLigne() {
        	boolean[] T = COULEURS.get(1);
        	
        	if(T[NOIR]) {
    			VITESSE = VMAX;
    			ANGLE   = utils.pid();
    			compteurblanc = 0;
    			coef = 4;
        	}
        	else {
    			VITESSE = 0.0;
    			if (compteurblanc > 25*tic) {
    				compteurblanc = 0;
    				coef = coef/2;
    				sens = -sens;        				
    			}
    			ANGLE = sens*Math.PI/coef;
        	}
        	
        	compteurblanc += tic;
        }
        
        public void ramasser(Piece b){
        	Point3d p = new Point3d();
        	b.getCoords(p);
        	b.moveToPosition(new Vector3d(p.x,p.y+1,p.z));
			sac.add(b);
        }
	
		public void deplacerObjets(){
			Piece b;
			Point3d p = new Point3d();
			for(int i = 0;i < sac.size();++i){
				b = sac.get(i);
				b.translateTo(new Vector3d(current_position.x-position.x,0,current_position.z-position.z));
			}
		}
    
		/******************************************************************************************************************************
		/******************************************************************************************************************************
		/***************************************************** MACHINE A ETAT *********************************************************
		/******************************************************************************************************************************
		/******************************************************************************************************************************/
		int etat,ramassePetitouGrand;
		boolean resteGrand,restePetit,aRamasse,plein;
		private final static int REPOSTOCHEMIN = 0;
		private final static int RECHERCHEPASTILLEJAUNE = 1;
		private final static int RECHERCHEPIECE = 2;
		private final static int RAMASSE = 3;
		private final static int RECHERCHEPASTILLEVERTE = 4;
		private final static int CHEMINTOREPOS = 5;
		private final static int DEPOSE = 6;
		private final static int FIN = 7;
		
		public void machineAEtat(){
			switch(etat){
				case REPOSTOCHEMIN:				reposToChemin();			break;
				case RECHERCHEPASTILLEJAUNE:	recherchePastilleJaune();	break;
				case RECHERCHEPIECE:			rechercherPiece();			break;
				case RAMASSE:					ramasse();					break;
				case RECHERCHEPASTILLEVERTE:	recherchePastilleVerte();	break;
				case CHEMINTOREPOS:				cheminToRepos();			break;
				case DEPOSE:					depose();					break;
				default:break;
			}
		}

		private void depose() {
			plein = false;
			if(restePetit){
				ramassePetitouGrand = 0;
				etat = REPOSTOCHEMIN;
			}
			else if(resteGrand){
				ramassePetitouGrand =1;
				etat = REPOSTOCHEMIN;
			}
			else
				etat = FIN;
		}


		private void cheminToRepos() {
			VITESSE = VMAX;
			ANGLE   = 0;
			
			//etat = DEPOSE;
			
		}




		private void recherchePastilleVerte() {
			boolean T[] = COULEURS.get(1);
			switch(internalState){
				case 0:
					suiviDeLigne();
					if(T[VERT]) {
						internalState++;
					}
				break;
				case 1:		
					if(roulerPendantDistance(0.3)) {
						internalState++;
					}
				break;
				case 2:
					if(aRamasse || plein) {
						if(pivoter(-Math.PI/4)) {
							internalState = 0;
							etat = CHEMINTOREPOS;
						}
					} else if(resteGrand) {
						if(pivoter(Math.PI/4)) {
							internalState = 0;							
							aRamasse = false;
							ramassePetitouGrand = 1;
							etat = RECHERCHEPASTILLEJAUNE;
						}
					}
				break;
			}	
		}




		private void ramasse() {
			/*aRamasse = true;
			if(ramassePetitouGrand == 1) {
				plein = true;
			}
			*/
			aRamasse = true;
			switch(internalState){
				case 0:
					if(pivoter(Math.PI/2)) {
						internalState++;
					}
				break;
				case 1:
					internalState = 0;
					etat = RECHERCHEPASTILLEJAUNE;
				break;
						
				default:
				break;
			}
			
			
		}




		private void rechercherPiece() {
			suiviDeLigne();
			if(rangeSensor.getFrontQuadrantMeasurement() < 1) {
				etat = RAMASSE;
			}
		}




		private void recherchePastilleJaune() {
			boolean T[] = COULEURS.get(1);
			switch(internalState){
				case 0:
					suiviDeLigne();
					if(T[JAUNE]) {
						internalState++;
					}
				break;
				case 1:		
					if(roulerPendantDistance(0.3)) {
						internalState++;
					}
				break;
				case 2:
					if(aRamasse) {
						if(pivoter(Math.PI/4)) {
							internalState++;							
						}						
					} else {
						if(pivoter(-Math.PI/4)) {
							internalState++;							
						}
					}
				break;
				case 3:
					if(roulerPendantDistance(0.2)) {
						internalState = 0;
						if(aRamasse) {
							etat = RECHERCHEPASTILLEVERTE;
						} else {
							etat = RECHERCHEPIECE;
						}
					}					
				break;					
				default:
				break;
			}
		}



		int internalState = 0;
		private void reposToChemin() {
			boolean T[] = COULEURS.get(1);
			switch(internalState){
				case 0: 
					if(T[VERT]) {
						internalState++;
					}
				break;
				case 1:
					if(roulerPendantDistance(0.4)) {
						internalState++;
					}
				break;
				case 2:
					if(pivoter(Math.PI/4)) {
						internalState = 0;
						etat = RECHERCHEPASTILLEJAUNE;
					}
				break;
			}
		}
		
		boolean roulerPendantDistanceActivated = false;
		Point3d position_save;
		private boolean roulerPendantDistance(double distance){
			if(!roulerPendantDistanceActivated){
				roulerPendantDistanceActivated = true;
				position_save = new Point3d(current_position);
			} else {
	            double d = Math.sqrt(Math.pow((current_position.x - position_save.x), 2) + Math.pow((current_position.z - position_save.z), 2));
	            if(d >= distance){
	            	roulerPendantDistanceActivated = false;
	            	return true;
	            }
			}
			return false;
		}
		
		public double compteur = 0;
		public boolean pivoter(double angle) {
			double alpha = 0.2; //Ya de la latence
			if(compteur < 1-alpha) {
				ANGLE = angle;
				VITESSE = 0;
				compteur += tic;
				return false;
			} else {
				ANGLE = 0;
				VITESSE = VMAX;
				compteur = 0;
				return true;
			}
		}
	}
	
	
	
	
    public DemoTp4() {
        light1IsOn = true;
        light2IsOn = true;
        setWorldSize(25);
        //Contours : wb = mur bas;wh = mur haut;md = mur droite; mg = mur gauche
        Wall wb = new Wall(new Vector3d(6.25, 0, 0), 25, 2, this);
        wb.rotate90(1);
        add(wb);
        Wall wd = new Wall(new Vector3d(0, 0, -12.50), 12.5f, 2.f, this);
        add(wd);
        Wall wh = new Wall(new Vector3d(-6.25, 0, 0), 25, 2, this);
        wh.rotate90(1);
        add(wh);
        Wall wg = new Wall(new Vector3d(0, 0, 12.50), 12.5f, 2.f, this);
        add(wg);
        
        //Entree
        Wall e1 = new Wall(new Vector3d(4.1, 0, -8.50), 4.25f, 2.f, this);
        add(e1);
        Wall e2 = new Wall(new Vector3d(-4.1, 0, -8.50), 4.25f, 2.f, this);
        add(e2);
        
        
        //Murs centraux
        Wall c1 = new Wall(new Vector3d(-1.45, 0, 0.1), 4f, 2.f, this);
        c1.rotate90(1);
        add(c1);
        Wall c2 = new Wall(new Vector3d(1.45, 0, 3.6), 4f, 2.f, this);
        add(c2);
        c2.rotate90(1);
        
        //Pastilles
        PastilleAgent p1 = new PastilleAgent(new Vector3d(0, 0.02, -6.4), new Color3f(0,0.74f,0), "demarrage", 0.25f, 30);
        add(p1);
        
        PastilleAgent p2 = new PastilleAgent(new Vector3d(3.95, 0.02, 0.05), new Color3f(0.9f,0.88f,0.14f), "demarrage", 0.25f, 30);
        add(p2);
        
        PastilleAgent p3 = new PastilleAgent(new Vector3d(-3.95, 0.02, 4.05), new Color3f(0.9f,0.88f,0.14f), "demarrage", 0.25f, 30);
        add(p3);
        
        //zones
        	//repos
        Box  zRep = new Box(new Vector3d(0,0.01,-11),new Vector3f(3,0,3),this, new Color3f(0.f,0.f,0.f));
        add(zRep);
        	//depot grands produits
        Box  zGrandProd = new Box(new Vector3d(-4.75,0.01,-11),new Vector3f(3,0,3),this, new Color3f(0.f,0.f,0.74f));
        add(zGrandProd);
        	//depot petits produits
        Box  zPetitProd = new Box(new Vector3d(4.75,0.01,-11),new Vector3f(3,0,3),this, new Color3f(0.74f,0.f,0.f));
        add(zPetitProd);
        
        	//usinage 1
        Box  zUsine1 = new Box(new Vector3d(0,0.01,-3.15),new Vector3f(3.3f,0,2.5f),this, new Color3f(255.f,255.f,255.f));
        add(zUsine1);
        	//usinage 2
        Box  zUsine2 = new Box(new Vector3d(0,0.01,6.85),new Vector3f(3.3f,0,2.5f),this, new Color3f(255.f,255.f,255.f));
        add(zUsine2);
        
        
        Box  pBas = new Box(new Vector3d(3.95,0.01,2.1),new Vector3f(0.2f,0,16.2f),this, new Color3f(0.f,0.f,0.f));
        Box  pHaut = new Box(new Vector3d(-3.95,0.01,2.1),new Vector3f(0.2f,0,16.2f),this, new Color3f(0.f,0.f,0.f));
        Box  pGauche = new Box(new Vector3d(0,0.01,10.6),new Vector3f(7.1f,0,0.2f),this, new Color3f(0.f,0.f,0.f));
        Box  pDroit = new Box(new Vector3d(0,0.01,-6.4),new Vector3f(7.1f,0,0.2f),this, new Color3f(0.f,0.f,0.f));
        Box  interGauche = new Box(new Vector3d(-1.1,0.01,4.05),new Vector3f(5.5f,0,0.2f),this, new Color3f(0.f,0.f,0.f));
        Box  interDroit = new Box(new Vector3d(1.1,0.01,0.05),new Vector3f(5.5f,0,0.2f),this, new Color3f(0.f,0.f,0.f));
        
        
        add(pBas);        
        add(pHaut);       
        add(pGauche);        
        add(pDroit);       
        add(interGauche);        
        add(interDroit);
       
        /*
        //Creations pièces
        //petites
        Box pp1 = new Box(new Vector3d(-1.275,0.01,0.05),new Vector3f(0.15f,0.38f,0.31f),this, new Color3f(1f,0.4f,0.f));
        add(pp1);
        Box pp2 = new Box(new Vector3d(-1.125,0.01,0.05),new Vector3f(0.15f,0.38f,0.31f),this, new Color3f(1f,0.4f,0.f));
        add(pp2);
        Box pp3 = new Box(new Vector3d(-0.975,0.01,0.05),new Vector3f(0.15f,0.38f,0.31f),this, new Color3f(1f,0.4f,0.f));
        add(pp3);
        
      //grandes
        Box gp1 = new Box(new Vector3d(1.275,0.01,4.05),new Vector3f(0.15f,0.55f,0.48f),this, new Color3f(0.6f,0.6f,0.6f));
        add(gp1);
        Box gp2 = new Box(new Vector3d(1.125,0.01,4.05),new Vector3f(0.15f,0.55f,0.48f),this, new Color3f(0.6f,0.6f,0.6f));
        add(gp2);
        Box gp3 = new Box(new Vector3d(0.975,0.01,4.05),new Vector3f(0.15f,0.55f,0.48f),this, new Color3f(0.6f,0.6f,0.6f));
        add(gp3);
        */

        add(new Robot(new Vector3d(0,0.1,-11), "robot 1"));

        
        //Creations pièces
        //petites
        
        Piece pp1 = new Piece(new Vector3d(-1.275,0.01,0.05),new Vector3f(0.15f,0.38f,0.31f),null, new Color3f(1f,0.4f,0.f));
        add(pp1);
        petites_pieces[0] = pp1;
        Piece pp2 = new Piece(new Vector3d(-1.125,0.01,0.05),new Vector3f(0.15f,0.38f,0.31f),null, new Color3f(1f,0.4f,0.f));
        add(pp2);
        petites_pieces[1] = pp2;
        Piece pp3 = new Piece(new Vector3d(-0.975,0.01,0.05),new Vector3f(0.15f,0.38f,0.31f),null, new Color3f(1f,0.4f,0.f));
        add(pp3);
        petites_pieces[2] = pp3;
        
        
      //grandes
        Piece gp1 = new Piece(new Vector3d(1.275,0.01,4.05),new Vector3f(0.15f,0.55f,0.48f),null, new Color3f(0.6f,0.6f,0.6f));
        add(gp1);
        grandes_pieces[0] = gp1;
        Piece gp2 = new Piece(new Vector3d(1.125,0.01,4.05),new Vector3f(0.15f,0.55f,0.48f),null, new Color3f(0.6f,0.6f,0.6f));
        add(gp2);
        grandes_pieces[1] = gp2;
        Piece gp3 = new Piece(new Vector3d(0.975,0.01,4.05),new Vector3f(0.15f,0.55f,0.48f),null, new Color3f(0.6f,0.6f,0.6f));
        add(gp3);
        grandes_pieces[2] = gp3;
        
        //Robot r2d2 = new Robot(new Vector3d(0,0.1,-11), "robot 1");
      //  add(r2d2);
        //r2d2.ramasser(petites_pieces[0]);
        genererCoins();
    }
    
    public class Point {
    	public double x,y;
    	public Point(double x, double y) {
    		this.x = x;
    		this.y = y;
    	}
    }
    
    private void genererCoins() {    	
    	List<Point> centres = new ArrayList<Point>();
    	
    	centres.add(new Point(3.6,-6.05));
    	centres.add(new Point(-3.6,-6.05));    	
    	centres.add(new Point(-3.6,10.25));
    	centres.add(new Point(3.6,10.25));
    	
    	double r = 0.35;
    	
    	double pas = 0.01;
    	
    	double x,y;
    	
    	double borne = Math.PI/2; 
    	
    	for(Point p : centres) {    		
    		for(double alpha=-borne+Math.PI/2; alpha>-borne; alpha -= pas){
        		x = p.x + r * Math.cos(alpha);
        		y = p.y + r * Math.sin(alpha);
        		Box tmp = new Box(new Vector3d(x,0.01,y),new Vector3f(0.2f,0,0.2f),this, new Color3f(0.f,0.f,0.f));
        		add(tmp);
        	}
    		borne = borne + Math.PI/2;
    	}
    	
    	
    }
}
