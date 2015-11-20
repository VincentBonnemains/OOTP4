package simbad.strategie;

public class StateMachine {
	/* etat = etat courant
	 * ramassePetitouGrand = 0 si on ramasse petites pieces, 1 si on ramasse grandes pieces
	 */
	int etat,ramassePetitouGrand;
	boolean resteGrand,restePetit,aRamasse,plein;
	
    public StateMachine() {
		super();
		this.etat = REPOSTOCHEMIN;
		this.ramassePetitouGrand = 0;
		this.resteGrand = true;
		this.restePetit = true;
		this.aRamasse = false;
		this.plein = false;
	}
    
    public void transition(){
		switch(etat){
		case REPOSTOCHEMIN:
			etat = RECHERCHEPASTILLEJAUNE;
			break;
		case RECHERCHEPASTILLEJAUNE :
			if(aRamasse)
				etat = RECHERCHEPASTILLEVERTE;
			else
				etat = RECHERCHEPIECE;
			break;
		case RECHERCHEPIECE:
			etat = RAMASSE;
			break;
		case RAMASSE :
			aRamasse = true;
			if(ramassePetitouGrand == 1)
				plein = true;
			etat = RECHERCHEPASTILLEJAUNE;
			break;
		case RECHERCHEPASTILLEVERTE :
			if(plein)
				etat = CHEMINTOREPOS;
			else{
				if(resteGrand){
					etat = RECHERCHEPASTILLEJAUNE;
					aRamasse = false;
					ramassePetitouGrand = 1;
				}
				else
					etat = CHEMINTOREPOS;
			}
			break;
		case CHEMINTOREPOS :
			etat = DEPOSE;
			break;
		case DEPOSE :
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
			break;
		default :
		}
	}
    
	private final static int REPOSTOCHEMIN = 0;
	private final static int RECHERCHEPASTILLEJAUNE = 1;
	private final static int RECHERCHEPIECE = 2;
	private final static int RAMASSE = 3;
	private final static int RECHERCHEPASTILLEVERTE = 4;
	private final static int CHEMINTOREPOS = 5;
	private final static int DEPOSE = 6;
	private final static int FIN = 7;
}
