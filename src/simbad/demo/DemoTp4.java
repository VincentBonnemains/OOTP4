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
import java.util.Random;

import javax.swing.JInternalFrame;
import javax.swing.JPanel;
import javax.vecmath.Color3f;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import simbad.gui.AgentInspector;
import simbad.sim.Agent;
import simbad.sim.Box;
import simbad.sim.CameraSensor;
import simbad.sim.PastilleAgent;
import simbad.sim.Piece;
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

	public DemoTp4 getInstance() {
		return this;
	}
	
	ArrayList<Piece> petites_pieces = new ArrayList<>();
	ArrayList<Piece> grandes_pieces = new ArrayList<>();
	public class Robot extends Agent {
		/*******************************************************
		 * ATTRIBUTES ******************************************
		 *******************************************************/
		private RobotUtils utils;
		final int GAUCHE=0,DROITE=1,NOIR=0,VERT=1,BLANC=2,JAUNE=3,ROUGE = 4, BLEU = 5,GRIS=6;
		int compteurPivotGauche=0,compteurPivotDroit=0;
		double elapsed;

		Point3d position,current_position;
		ArrayList<Piece> petit_sac = new ArrayList<Piece>();
		ArrayList<Piece> gros_sac  = new ArrayList<Piece>();
        double tic = 0.01,tac=0;
        double xoffset_p;
        double xoffset_g;

		double ANGLE = 0;
		public double VMAX = 1.4;
		double VMIN = 0.2;
		double VITESSE = VMAX;
		double compteurblanc = 0.0;
    	double coef = 8;
    	int sens = 1;
    	public int nbColors = 7;

    	boolean frontColors[] = new boolean[nbColors];
		boolean backColors[]  = new boolean[nbColors];
		int ramassage = 0,depot = 0,etapRamassage = 0;
		public boolean retrouverChemin = false;
		
		public CameraSensor frontSensor,backSensor;
		
		public BufferedImage frontMatrix,backMatrix;
		RangeSensorBelt rangeSensor;
				
		JPanel frontSensorPanel,backSensorPanel;
        JInternalFrame window;
        
        DemoTp4 demo;
        /*******************************************************
         * END ATTRIBUTES **************************************
         *******************************************************/
        
        
        public Robot(DemoTp4 demo,Vector3d position, String name) {
            super(position, name);
            this.demo = demo;
            utils = new RobotUtils(this);
            //sensors = RobotFactory.addCameraBeltSensor(this,sensors);
            backSensor  = RobotFactory.addBackCameraSensor(this);
            frontSensor = RobotFactory.addFrontCameraSensor(this);
            rangeSensor = RobotFactory.addSonarBeltSensor(this);
            this.position = new Point3d();
            this.current_position = new Point3d();
            // prepare a buffer for storing image
            frontMatrix = frontSensor.createCompatibleImage();
            backMatrix  = backSensor.createCompatibleImage();
           
            // Prepare UI panel for image display
            frontSensorPanel = new ImagerPanel(this);
            Dimension dim = new Dimension(frontMatrix.getWidth(),frontMatrix.getHeight());
            frontSensorPanel.setPreferredSize(dim);
            frontSensorPanel.setMinimumSize(dim);
           
            setUIPanel(frontSensorPanel);
           
         // Prepare UI panel for image display
          //  backSensorPanel = new ImagerBackSensorPanel(this);
        //    backSensorPanel .setPreferredSize(dim);
         //   backSensorPanel .setMinimumSize(dim);
           // setUIPanel(backSensorPanel);
            
            //Machine a etat variable
            this.etat = REPOSTOCHEMIN;
    		this.ramassePetitouGrand = 0;
    		this.resteGrand = true;
    		this.restePetit = true;
    		this.aRamasse = false;
    		this.plein = false;
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
        	
        	tac = getLifeTime() - elapsed;
            if (tac >= tic) {            	
	            getCoords(current_position);
				machineAEtat();
            	setTranslationalVelocity(VITESSE*(1-ramassage*0.05));
	            setRotationalVelocity(ANGLE);
	            /* Mettre � jour les capteurs */
            	frontSensor.copyVisionImage(frontMatrix);
            	frontColors = utils.colorsDetectedFront(frontMatrix);
            	frontSensorPanel.repaint();
            	
            	backSensor.copyVisionImage(backMatrix);
            	backColors = utils.colorsDetectedBack(backMatrix);
            	
            	/*backSensorPanel.repaint();*/
            	/* ***************************** */
            	getCoords(position);
            	elapsed = getLifeTime();
            }            
            if (collisionDetected()) {
            	setTranslationalVelocity(0);
            }
        }
    	
        public void suiviDeLigne() {
        	boolean[] T = frontColors;
        	
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
        
        
        public void ramasser(Piece piece) {     	
        	Vector3d pos;
        	Point3d position = new Point3d();
        	piece.getCoords(position);
        	if(piece.type == 0) {
        		petit_sac.add(piece);
        		xoffset_p = 0.5 + petit_sac.size() * (petit_sac.get(0).getRadius()+0.1);
	        	pos = new Vector3d(xoffset_p,position.y-(this.getHeight()/2),0);
	         	
	         	addPieceDevice(piece,pos,0);
        	}
        	else {
        		gros_sac.add(piece);
        		if(petit_sac.size() == 0) {
        			xoffset_p = 0.5;
        		} 
    			xoffset_g = gros_sac.size() * (gros_sac.get(0).getRadius()+0.1);
        		pos = new Vector3d(xoffset_p + xoffset_g,position.y-(this.getHeight()/2),0); 
             	addPieceDevice(piece,pos,0);
        	}
        }
        
        public void deposer(Piece piece) {
        	Point3d posPieceRelative = new Point3d();
        	Point3d posRobotAbsolu   = new Point3d();
        	piece.getCoords(posPieceRelative);
        	getCoords(posRobotAbsolu);
        	
        	
        	
       	
        	double x = posRobotAbsolu.x + posPieceRelative.x*(int) Math.signum(posRobotAbsolu.x);
        	double y = posRobotAbsolu.y;// + posPieceRelative.y;
        	double z = posRobotAbsolu.z + posPieceRelative.z*(int) Math.signum(posRobotAbsolu.z);
        	/*System.out.println(posRobotAbsolu.x+" "+posRobotAbsolu.y+" "+posRobotAbsolu.z);
        	System.out.println(posPieceRelative.x+" "+posPieceRelative.y+" "+posPieceRelative.z);
        	System.out.println(x+" "+y+" "+z);*/
        	
        	removePieceDevice(piece);
        	piece.moveToPosition(new Vector3d(x,0,z));
        	demo.add(piece);
        	piece.attach();
        	ramassage--;        	
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
		private final static int RECHERCHEREPOS = 5;
		private final static int REPOSTOZONEPP = 8;
		private final static int REPOSTOZONEGP = 9;
		private final static int DEPOSE = 6;
		private final static int FIN = 7;
		
		public void machineAEtat(){
			switch(etat){
				case REPOSTOCHEMIN:				reposToChemin();			break;
				case RECHERCHEPASTILLEJAUNE:	recherchePastilleJaune();	break;
				case RECHERCHEPIECE:			rechercherPiece();			break;
				case RAMASSE:					ramasse();					break;
				case RECHERCHEPASTILLEVERTE:	recherchePastilleVerte();	break;
				case RECHERCHEREPOS:			rechercheRepos();			break;
				case REPOSTOZONEPP:				reposToZonePP();			break;
				case REPOSTOZONEGP:				reposToZoneGP();			break;
				case DEPOSE:					depose();					break;
				case FIN:						fin();						break;
				default:break;
			}
		}
		
		public String getState(){
			switch(etat){
			case REPOSTOCHEMIN:				return("reposToChemin");
			case RECHERCHEPASTILLEJAUNE:	return("recherchePastilleJaune");
			case RECHERCHEPIECE:			return("recherchePiece");
			case RAMASSE:					return("ramasse");
			case RECHERCHEPASTILLEVERTE:	return("recherchePastilleVerte");
			case RECHERCHEREPOS:			return("rechercheRepos");
			case REPOSTOZONEPP:				return("reposToZonePP");
			case REPOSTOZONEGP:				return("reposToZoneGP");
			case DEPOSE:					return("depose");
			case FIN:						return("fin");
			default: return("don't know");
		}
		}
		
		private void fin(){
			VITESSE = 0;
			ANGLE = 0;
		}

		private void depose() {			
			switch(internalState){
				//DESCENTE
				case 0:
					plein = false;
					VITESSE = 0;
					if(etapRamassage <= 8){
						if(petit_sac.size() != 0){
							for(Piece p:petit_sac){
								p.translateTo(new Vector3d(0,-0.1,0));
							}
							for(Piece p:gros_sac)
								p.translateTo(new Vector3d(0,-0.1,0));
						}
						else{
							for(Piece p:gros_sac){
								p.translateTo(new Vector3d(0,-0.1,0));
							}
						}
						++etapRamassage;
					}
					else{
						etapRamassage = 0;
						internalState++;
					}
				break;
				case 1:
					if(petit_sac.size() > 0) {
						for(Piece p:petit_sac)	deposer(p);
						petit_sac.clear();
					} else {
						for(Piece p:gros_sac)  deposer(p);							
						gros_sac.clear();
					}
					internalState++;
				break;
				case 2:
					if(etapRamassage <= 8){
						for(Piece p:gros_sac)
							p.translateTo(new Vector3d(0,0.1,0));
						etapRamassage++;
					}
					else{
						etapRamassage = 0;
						internalState++;
					}
				break;
				case 3:
					if(pivoter(Math.PI))
						internalState++;
					break;
				case 4:
					plein = false;
					internalState = 0;
					etat = RECHERCHEREPOS;
					break;
				}
		}


		private void rechercheRepos() {
			VITESSE = VMAX;
			ANGLE   = 0;
			boolean[] T = frontColors;
			switch(internalState){
				case 0:
					if(T[NOIR]) {
						internalState++;
					}
				break;
				case 1:
					if(roulerPendantDistance(1.5)) {
						internalState++;
					}
				break;
				case 2:
					if(petit_sac.size() > 0) { //HAUT ET GAUCHE
						if(pivoter(-Math.PI/2)) {
							internalState = 0;
							etat = REPOSTOZONEPP;
						}
					} else if(petit_sac.size() == 0 && restePetit) { //Chemin de petite zone vers grande zone		DE DROITE A GAUCHE	
						restePetit = false;
						internalState = 0;
						etat = REPOSTOZONEGP;						
					} else if(gros_sac.size() == 0 && resteGrand){
						if(pivoter(-Math.PI/2)){
							internalState = 0;
							etat = REPOSTOCHEMIN;
						}
					} else if(gros_sac.size() > 0) {
						if(pivoter(Math.PI/2)) {
							internalState = 0;
							etat = REPOSTOZONEGP;
						}
					} else {
						internalState = 0;
						etat = FIN;
					}				
				break;
			}
			
		}

		private void reposToZonePP() {
			boolean[] T = frontColors;
			switch(internalState){
				case 0:
					if(T[ROUGE]) {
						internalState++;
					}
				break;
				case 1:
					if(roulerPendantDistance(1)) {
						internalState++;
					}
				break;
				case 2:
					internalState = 0;
					etat = DEPOSE;
				break;
			}
			
		}
		
		private void reposToZoneGP() {
			/*VITESSE = VMAX;
			ANGLE   = 0;*/
			boolean[] T = frontColors;
			switch(internalState){
				case 0:
					if(T[BLEU]) {
						internalState++;
					}
				break;
				case 1:
					if(roulerPendantDistance(1)) {
						internalState++;
					}
				break;
				case 2:
					internalState = 0;
					etat = DEPOSE;
				break;
			}
			
		}


		private void recherchePastilleVerte() {
			boolean[] T = frontColors;
			
			switch(internalState){
				case 0:
					suiviDeLigne();
					if(T[VERT]) {
						internalState++;
					}
				break;
				case 1:
					if(plein){
						if(roulerPendantDistance(0.3)) {
							internalState++;
						}
					} else {
						internalState = 0;
						etat = RECHERCHEPASTILLEJAUNE;
					}
				break;
				case 2:
					if(pivoter(Math.PI/2)) {
						internalState++;
					}						
				break;
				case 3:
					if(roulerPendantDistance(0.5)){
						internalState = 0;
						etat = RECHERCHEREPOS;
					}
					break;
			}	
		}




		private void ramasse() {
			VITESSE = 0;
			Random rand = new Random();
			int j;
			switch(internalState){
				//depot
				case 0:
					if((ramassePetitouGrand == 1) && etapRamassage <= 8){
						for(Piece p:petit_sac){
							p.translateTo(new Vector3d(0,-0.1,0));
						}
						++etapRamassage;
					}
					else{
						etapRamassage = 0;
						internalState++;
					}
				break;
				//elevation
				case 1:
					if(etapRamassage <= 8){
						switch(ramassePetitouGrand){
						case 0:
							for(Piece p:petites_pieces){
								p.translateTo(new Vector3d(0,0.1,0));
							}
							break;
						case 1:
							if(etapRamassage == 0){
								j = (rand.nextInt(3))+1;
								if(j > grandes_pieces.size())
									j = grandes_pieces.size();
								for(int i =0;i < j;++i)
									gros_sac.add(grandes_pieces.get(i));
							}
							for(Piece p:gros_sac){
								p.translateTo(new Vector3d(0,0.1,0));
							}
							for(Piece p:petit_sac)
								p.translateTo(new Vector3d(0,0.1,0));
							break;
						}
						etapRamassage++;
					}
					else{
						etapRamassage = 0;
						internalState++;
					}
				break;
				case 2:
					aRamasse = true;
					if(ramassePetitouGrand == 1) {
						plein = true;
					}
						switch(ramassePetitouGrand){
							case 0: //On ramasse les petites
								for(Piece p:petites_pieces){
									ramasser(p);									
									ramassage++;
								}
								petites_pieces.clear();
							break;
							case 1://On ramasse les grandes								
								j = gros_sac.size();
								gros_sac.clear();
								for(int i=0; i<j; i++){
									ramasser(grandes_pieces.get(0));
									grandes_pieces.remove(0);
									ramassage++;
								}
							break;
							default:
								break;
					}
					internalState++;
				break;
				
				case 3:
					if(pivoter(Math.PI)) {
						//double d = zoffset_p + zoffset_g + 0.15;
						if(ramassePetitouGrand == 1) {							
							if(backColors[GRIS] == true)
								resteGrand = true;
							else
								resteGrand = false;
							}							
						internalState++;
					}
				break;
				case 4:
					aRamasse = true;
					internalState = 0;
					if(ramassePetitouGrand == 1)
						plein = true;
					etat = RECHERCHEPASTILLEJAUNE;
				break;
						
				default:
			}
			
			
		}




		private void rechercherPiece() {
			suiviDeLigne();
			
			double d = rangeSensor.getFrontQuadrantMeasurement();
			if(petit_sac.size() == 0 && d < 1) {
				internalState = 0;
				etat = RAMASSE;
			} else if(restePetit && d < xoffset_p + 0.2) { //Ramassage des grosses pi�ces quand on a deja 3 petites pieces, offset_p deja calcul�
				internalState = 0;
				etat = RAMASSE;
			}
		}




		private void recherchePastilleJaune() {
			boolean[] T = frontColors;

			switch(internalState){
				case 0:
					suiviDeLigne();
					if(T[JAUNE]) {
						internalState++;
					}
				break;
				case 1:
					if(roulerPendantDistance(0.25)) {
						internalState++;
					}
				break;
				case 2:
					if(aRamasse) {
						if(ramassePetitouGrand == 0){
							if(pivoter(Math.PI/2)) {
								internalState++;							
							}
						}
						else{
							if(pivoter(-Math.PI/2)) {
								internalState++;							
							}
						}
					} else {
						if(ramassePetitouGrand == 0){
							if(pivoter(-Math.PI/2)) {
								internalState++;							
							}
						}
						else{
							if(pivoter(Math.PI/2)) {
								internalState++;							
							}
						}
					}
				break;
				case 3:
					if(aRamasse) {
						internalState = 0;
						aRamasse = false;
						if(ramassePetitouGrand == 0)
							ramassePetitouGrand = 1;
						etat = RECHERCHEPASTILLEVERTE;
					} else {
						internalState = 0;
						etat = RECHERCHEPIECE;
					}
									
				break;					
				default:
				break;
			}
		}



		int internalState = 0;
		private void reposToChemin() {
			
			boolean[] T = frontColors;
			
			switch(internalState) {
				case 0:
					if(roulerPendantDistance(2)){ //Le temps de quitter la zone
						internalState++;
					}
				break;
				case 1: 
					if(T[VERT] || T[NOIR]) { //Vert ou noir si jamais on arrive de travers
						internalState++;
					}
				break;
				case 2:
					if(roulerPendantDistance(0.4)) {
						internalState++;
					}
				break;
				case 3:
					if(ramassePetitouGrand == 0){
						if(pivoter(Math.PI/2)) {
							internalState = 0;
							etat = RECHERCHEPASTILLEJAUNE;
						}
					}
					else{
						if(pivoter(-Math.PI/2)) {
							internalState = 0;
							etat = RECHERCHEPASTILLEJAUNE;
						}
					}
				break;
			}
		}
		
		boolean roulerPendantDistanceActivated = false;
		Point3d position_save;
		private boolean roulerPendantDistance(double distance){
			if(!roulerPendantDistanceActivated){
				ANGLE = 0;
				VITESSE = VMAX;
				roulerPendantDistanceActivated = true;
				position_save = new Point3d(current_position);
			} else {
				this.getCoords(current_position);
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
			ANGLE = angle;
			VITESSE = 0;
			compteur += tac;
			if(compteur >= Math.abs(angle/getRotationalVelocity())) {
				ANGLE = 0;
				VITESSE = VMAX;
				compteur = 0;
				return true;
			}			
				
			return false;
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
        //Creations pi�ces
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

        add(new Robot(this,new Vector3d(0,0.1,-11), "robot 1"));

        
        //Creations pi�ces
        //petites
        
        Piece pp1 = new Piece(new Vector3d(-1.275,0.01,0.05),new Vector3f(0.15f,0.38f,0.31f),null, new Color3f(1f,0.4f,0.f),0);
        add(pp1);
        petites_pieces.add(pp1);
        Piece pp2 = new Piece(new Vector3d(-1.125,0.01,0.05),new Vector3f(0.15f,0.38f,0.31f),null, new Color3f(1f,0.4f,0.f),0);
        add(pp2);
        petites_pieces.add(pp2);
        Piece pp3 = new Piece(new Vector3d(-0.975,0.01,0.05),new Vector3f(0.15f,0.38f,0.31f),null, new Color3f(1f,0.4f,0.f),0);
        add(pp3);
        petites_pieces.add(pp3);
        
        
      //grandes
        Piece gp1 = new Piece(new Vector3d(1.275,0.01,4.05),new Vector3f(0.15f,0.55f,0.48f),null, new Color3f(0.6f,0.6f,0.6f),1);
        add(gp1);
        Piece gp2 = new Piece(new Vector3d(1.125,0.01,4.05),new Vector3f(0.15f,0.55f,0.48f),null, new Color3f(0.6f,0.6f,0.6f),1);
        add(gp2);
        Piece gp3 = new Piece(new Vector3d(0.975,0.01,4.05),new Vector3f(0.15f,0.55f,0.48f),null, new Color3f(0.6f,0.6f,0.6f),1);
        add(gp3);
        grandes_pieces.add(gp3);
        grandes_pieces.add(gp2);
        grandes_pieces.add(gp1);
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
    		for(double alpha=-borne+Math.PI/2; alpha>-borne; alpha -= pas) {
        		x = p.x + r * Math.cos(alpha);
        		y = p.y + r * Math.sin(alpha);
        		Box tmp = new Box(new Vector3d(x,0.01,y),new Vector3f(0.2f,0,0.2f),this, new Color3f(0.f,0.f,0.f));
        		add(tmp);
        	}
    		borne = borne + Math.PI/2;
    	}
    	
    	
    }
}
