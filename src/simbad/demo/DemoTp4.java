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

import javax.vecmath.Color3f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import simbad.demo.BumpersDemo.Robot;
import simbad.sim.Agent;
import simbad.sim.Arch;
import simbad.sim.Box;
import simbad.sim.CameraSensor;
import simbad.sim.EnvironmentDescription;
import simbad.sim.PastilleAgent;
import simbad.sim.RangeSensorBelt;
import simbad.sim.RobotFactory;
import simbad.sim.Wall;


/**
 * The Base class for all demos.
 */
public class DemoTp4 extends Demo {
    
	public class Robot extends Agent {
        RangeSensorBelt sonars;
        CameraSensor camera;
        public Robot (Vector3d position, String name) {     
            super(position,name);
            // Add sensors
            camera = RobotFactory.addCameraSensor(this);
            RobotFactory.addBumperBeltSensor(this);
           // Add sonars and get corresponding object.
           sonars  = RobotFactory.addSonarBeltSensor(this);
          
        }
        
        /** Initialize Agent's Behavior*/
        public void initBehavior() {
            // nothing particular in this case
        }
        
        /** Perform one step of Agent's Behavior */
        public void performBehavior() {

          
                // progress at 0.5 m/s
                setTranslationalVelocity(0.5);
                // frequently change orientation 
                if ((getCounter() % 100)==0) setRotationalVelocity(Math.PI/2 * (0.5 - Math.random()));
            
            // print front sonar every 100 frames
            if(getCounter() %100==0)
                System.out.println("Sonar num 0  = "+sonars.getMeasurement(0));
           
        }
    }
    public DemoTp4() {
        light1IsOn = true;
        light2IsOn = false;
        setWorldSize(30);
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
        add(new Robot(new Vector3d(0, 0, 0), "robot 1")); 
        
        //Parcours
        Box  pBas = new Box(new Vector3d(3.95,0.01,2.1),new Vector3f(0.2f,0,16.9f),this, new Color3f(0.f,0.f,0.f));
        add(pBas);
        Box  pHaut = new Box(new Vector3d(-3.95,0.01,2.1),new Vector3f(0.2f,0,16.9f),this, new Color3f(0.f,0.f,0.f));
        add(pHaut);
        Box  pGauche = new Box(new Vector3d(0,0.01,10.6),new Vector3f(8.1f,0,0.2f),this, new Color3f(0.f,0.f,0.f));
        add(pGauche);
        Box  pDroit = new Box(new Vector3d(0,0.01,-6.4),new Vector3f(8.1f,0,0.2f),this, new Color3f(0.f,0.f,0.f));
        add(pDroit);
        Box  interGauche = new Box(new Vector3d(-1.1,0.01,4.05),new Vector3f(5.5f,0,0.2f),this, new Color3f(0.f,0.f,0.f));
        add(interGauche);
        Box  interDroit = new Box(new Vector3d(1.1,0.01,0.05),new Vector3f(5.5f,0,0.2f),this, new Color3f(0.f,0.f,0.f));
        add(interDroit);
    }

}
