package se.oru.coordination.coordination_oru.ourproject.models;

import se.oru.coordination.coordination_oru.ourproject.algorithms.*;

public class VehicleThread implements Runnable {
	
	private Vehicle v;
	
	public VehicleThread( Vehicle v){
		this.v = v;
	}
	
	// MAIN ALGORITHM
	public void run() {
		int i = 0;
		String List;
		CalcoloPrecedenze CompPrec = new CalcoloPrecedenze();
		v.setPathIndex(0);
		//System.out.println("\n" + this.v.getSpatialEnvelope().getPolygon().getEnvelope());

		try{
			while(v.getPathIndex()< v.getSpatialEnvelope().getPath().length){		// this will be while true
				List = " ";
				
				v.clearCs();
				for (Vehicle vh : this.v.getNears()){
					v.appendCs(vh);
					List = (List + vh.getID()+" " );
				}

				Boolean prec = true;
				
				for (CriticalSection cs : this.v.getCs()){ //devo averle ordinate
					//prec = CompPrec.CalcoloPrecedenze(cs);
					prec = CompPrec.CalcoloPrecedenze(this.v.getCs().get(0)); //momentaneamente calcolo solo la prima
				}
				

				v.moveVehicle(prec);
				


				
				int nCs;
				String CsString;
				if (v.getCs().size() != 0){
					nCs = v.getCs().size();
					CsString = ("Inizio Cs " + nCs +" mia: " + v.getCs().get(0).getTe1End()+ "\tCs altrui: " + v.getCs().get(0).getTe2End());
				}
				else {
					nCs = 0;
					CsString = ("Cs " + nCs );
				}
				
				System.out.println("\n R" + this.v.getID() + " : \n" + 
					"List : "  + List + "\n" + 
					"precedeza :" + prec + "\n" + 
					"PathIndx : " 	+ v.getPathIndex() + "\n" + 
					CsString	);
				
				i++;
				Thread.sleep(v.getTc());
				
			}
			System.out.println("\n R" + this.v.getID() + " : GOAL RAGGIUNTO" );
		}
		catch (InterruptedException e) {
			System.out.println("Thread interrotto");
		}
		
	}

}

