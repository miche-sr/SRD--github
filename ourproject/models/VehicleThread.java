package se.oru.coordination.coordination_oru.ourproject.models;

public class VehicleThread implements Runnable {
	
	private Vehicle v;
	private double elapsedTrackingTime = 0.0;
	
	public VehicleThread(Vehicle v){
		this.v = v;
	}
	
	// MAIN ALGORITHM
	public void run() {
		String List;
		//v.setPathIndex(0);
		//System.out.println(v.getSecForSafety());
		try{
			while(v.getPathIndex() < v.getWholePath().length){		// this will be while true
				v.setMyTimes();
				v.setTrajectoryEnvelope();
				Thread.sleep(v.getTc());
				this.elapsedTrackingTime =+ v.getTc();

				//System.out.println(Arrays.toString(v.getMyTimes()));
				//System.out.println(v.getSpatialEnvelope().getPath().length);
								
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
				v.setStoppingPoint();
				if (v.getStoppingPoint() == v.getCriticalPoint())
					v.setSlowingPoint(v.getPathIndex());
				
				v.setPathIndex(elapsedTrackingTime);
				v.setPose(v.getWholePath()[v.getPathIndex()].getPose());
				//v.moveVehicle(prec);
				
				//printLog(List, prec);
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
				"\t R" + cs.getVehicle2().getID() + ":" +
				"\t" + cs.getTe2Start()+"-"+cs.getTe2End()) + "\n";
				i += 1;
			}
		}
		else
			CsString = ("0 Sezioni Critiche");
		
		System.out.println("\n R" + this.v.getID() + " : \n" + 
			"Vicini: "  + List + "\n" + 
			"Precedenza: " + prec + "\n" + 
			"Path Index: " 	+ v.getPathIndex() + "\n" +
			"Critical Point: " + v.getCriticalPoint() + "\n" +
			CsString
			);
	}
}

