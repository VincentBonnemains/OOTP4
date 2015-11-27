package examples;


import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import simbad.demo.DemoTp4;
import simbad.gui.Simbad;
import simbad.sim.*;



/**
  Derivate your own code from this example.
 */


public class Example1 {

   

    public static void main(String[] args) {
        // request antialising
        System.setProperty("j3d.implicitAntialiasing", "true");
        // create Simbad instance with given environment
        Simbad frame = new Simbad(new DemoTp4(), false);
    }

} 