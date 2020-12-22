package se.oru.coordination.coordination_oru.ourproject.models;

import java.sql.Time;

public class VehicleThread implements Runnable {
	
	private Vehicle v;
	private double elapsedTrackingTime = 0.0;
	private int cp = -2;
	
	public VehicleThread(Vehicle v){
		this.v = v;

	}
	
	
	// MAIN ALGORITHM
	public void run() {
		String List;
		//v.setPathIndex(0);
		//System.out.println(v.getSecForSafety());
		v.setTimes();
		v.setSpatialEnvelope(); // in questo modo si sta calcolando setTime e seTspatial come se non ci fossero altri robot
		
		//Thread.sleep(v.getTc());
		
		// infatti volevo mettere uno sleep dato tipo con time
		// questo sleep non gli piace, non so se serva effettivamente uno sleep
		
		
		
		

		try{
			while(v.getPathIndex() < v.getWholePath().length-1){		// this will be while true
				//v.setMyTimes();

				//v.setSpatialEnvelope();
				// v.setTimes();
				// v.setSpatialEnvelope();

								
				v.clearCs();
				List = "";
				for (Vehicle vh : this.v.getNears()){
					v.appendCs(vh);
					List = (List + vh.getID() + " " );
				}
				
				Boolean prec = true;
				v.setCriticalPoint(-1);
				for (CriticalSection cs : this.v.getCs()){
					prec = cs.ComputePrecedences();
					if (prec == false)
						v.setCriticalPoint(cs);
						break;
				}
				
				//if (cp != v.getCriticalPoint()){
					cp = v.getCriticalPoint();
					v.setSlowingPoint();
					v.setTimes();
				//}
				
				//System.out.print(v.getTimes() + "\n");
				//System.out.print(  "\n" +v.getTruncateTimes() +   "\n");

				
				v.setPathIndex(elapsedTrackingTime);
				v.setPose(v.getWholePath()[v.getPathIndex()].getPose());
				v.setStoppingPoint();
				//v.moveVehicle(prec);
				v.setSpatialEnvelope();

				printLog(List, prec);

				Thread.sleep(v.getTc());
				this.elapsedTrackingTime += v.getTc()*Vehicle.mill2sec;
			}
			System.out.println("\n R" + this.v.getID() + " : GOAL RAGGIUNTO" );
		}
		
		catch (InterruptedException e) {
			System.out.println("Thread interrotto");

		}	
	}



	
	public void printLog(String List, Boolean prec) {
		String CsString = "";
		String Dist = String.format("%.2f", v.getDistanceTraveled());
		String Vel = String.format("%.2f", v.getVelocity());
		String infoCs = this.infoCs();
		if (v.getCs().size() != 0){
			int i = 0;
			for (CriticalSection cs : v.getCs()) {
				CsString = CsString + (i+1 +"° Sezione Critica" +
				"\t Mia: " + cs.getTe1Start()+"-"+cs.getTe1End() +
				"\t R" + cs.getVehicle2().getID() + ":" +
				"\t" + cs.getTe2Start()+"-"+cs.getTe2End()) + "\n";
				i += 1;
			}
		}
		else	CsString = ("0 Sezioni Critiche");
				
		System.out.println(
			"\n============================================================\n"+
			"\nInfo R" + this.v.getID() + " : \n" + 
			"Last Index: "+ v.getWholePath().length + "\t \t " + infoCs + "\n" + 
			"Vicini: "  + List + "\t \t Precedenza: " + prec +  "\n" + 
			"Path Index: " 	+ v.getPathIndex() + "\t \t Stopping Point: " + v.getStoppingPoint() + "\n" +
			"Distance: "+ Dist  + "\t \t Velocity: " + Vel + "\n" +
			"SLowing Point: " + v.getSlowingPoint() +"\t Critical Point:" + v.getCriticalPoint() + "\n" + 
			CsString + "\n \n" +
			"Percorso Trasmesso \n" +v.getTruncateTimes() +   "\n"
			);
	}
	public String infoCs() {
		
		String infoCs;
		int cp = v.getCriticalPoint();
		if (cp == -1) cp = v.getWholePath().length;
		

		if (v.getPathIndex() > v.getSlowingPoint() && v.getVelocity()>=v.getAccMAx())
			infoCs = "Slowing";
		else if(v.getPathIndex() > v.getSlowingPoint() && v.getVelocity()<v.getAccMAx())
			infoCs = "Min Velocity";
		else 
			infoCs = "Moving on";
		
		if (v.getPathIndex() == cp && v.getVelocity() == 0)
			infoCs = "Waiting";
		if (v.getPathIndex() > cp)
			infoCs = "ATTENTION ROBOT STOPPED AFTER CRITICAL POINT";
		return infoCs;
	}



}