package se.oru.coordination.coordination_oru.ourproject.models;

public class VehicleThread implements Runnable {
	
	private Vehicle v;
	private double elapsedTrackingTime = 0.0;
	private boolean arrived = false;

	public VehicleThread(Vehicle v){
		this.v = v;
	}
	
	// MAIN ALGORITHM
	public void run() {
		String List;
		Boolean prec;
		
		try{
			while(true){ //v.getPathIndex() <= v.getWholePath().length-1
				v.setMyTimes();
				v.setTrajectoryEnvelope();
				
				Thread.sleep(v.getTc());
								
				/********INIZIO INTERVALLO K********/	// comincio a stampare da t0
				v.clearCs();
				List = "";
				for (Vehicle vh : this.v.getNears()){
					v.appendCs(vh);
					List = (List + vh.getID() + " " );
				}
				
				prec = true;
				v.setCriticalPoint(-1);
				for (CriticalSection cs : this.v.getCs()){
					prec = cs.ComputePrecedences();
					if (prec == false)
						v.setCriticalPoint(cs);
						break;
				}
				v.setStoppingPoint();
				if (arrived == true) break;
				if (v.getCriticalPoint() == -1)
					v.setSlowingPoint(1000);
				if (v.getStoppingPoint() == v.getCriticalPoint()|| v.getStoppingPoint() == v.getWholePath().length-1)
					v.setSlowingPoint(v.getDistanceTraveled());
				
				printLog(List, prec);
				
				/*********FINE INTERVALLO K*********/
				this.elapsedTrackingTime += v.getTc()*Vehicle.mill2sec;
				// valori del nuovo intervallo K+1, ovvero quelli alla fine del precedente,
				// valori che assumerà nella realtà dopo il tempo di sleep
				arrived = v.setPathIndex(elapsedTrackingTime); 
				v.setPose(v.getWholePath()[v.getPathIndex()].getPose());

			}
			System.out.println("\n R" + this.v.getID() + " : GOAL RAGGIUNTO" );
		}
		
		catch (InterruptedException e) {
			System.out.println("Thread interrotto");

		}	
	}

	
	public void printLog(String List, Boolean prec) {
		String CsString = "";
		if (v.getCs().size() != 0){
			int i = 0;
			for (CriticalSection cs : v.getCs()) {
				CsString = CsString + (i+1 +"° Sezione Critica" +
				"\t Mia: " + cs.getTe1Start()+"-"+cs.getTe1End() +
				"\t R" + cs.getVehicle2().getID() +
				": " + cs.getTe2Start()+"-"+cs.getTe2End()) + "\n";
				i += 1;
			}
		}
		else
			CsString = ("0 Sezioni Critiche\n");
		String dist = String.format("%.2f", v.getDistanceTraveled());
		String slow = String.format("%.2f", v.getSlowingPoint());
		String vel = String.format("%.2f", v.getVelocity());

		System.out.println("\n R" + this.v.getID() + " : \n" + 
			"Robot Vicini: "  + List + "\n" + 
			CsString +
			"Precedence: " + prec +  "\n" + 
			"Velocity: " + vel + "\n" +
			"Stopping Point: " + v.getStoppingPoint() + "\t Critical Point: " + v.getCriticalPoint() + "\n" +
			"Path Index: " + v.getPathIndex() + "\t\t Last Index: " + (v.getWholePath().length-1) + "\n" +
			"Distance Traveled: " + dist + "\t Slowing Point: " + slow + "\n"
			);
	}
}