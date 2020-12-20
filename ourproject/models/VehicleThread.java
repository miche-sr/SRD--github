package se.oru.coordination.coordination_oru.ourproject.models;

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
		
		try{
			while(v.getPathIndex() < v.getWholePath().length-1){		// this will be while true
				v.setMyTimes();

				//v.setSpatialEnvelope();
				

								
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
					v.setTimes();
				//}
				v.setSpatialEnvelope();
				//System.out.print(v.getTimes() + "\n");
				//System.out.print(v.getTruncateTimes());

				v.setSlowingPoint();
				v.setPathIndex(elapsedTrackingTime);
				v.setPose(v.getWholePath()[v.getPathIndex()].getPose());
				v.setStoppingPoint();
				//v.moveVehicle(prec);
				
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
				
		System.out.println("\n R" + this.v.getID() + " : \n" + 
			"Vicini: "  + List + "\n" + 
			"Path Index: " 	+ v.getPathIndex() + "\t \t Stopping Point: " + v.getStoppingPoint() + "\n" +
			"Distance: "+ Dist  + "\t \t Velocity: " + Vel + "\n" +
			"Critical Point:" + v.getCriticalPoint() + "\t Last Index: "+ v.getWholePath().length + "\n" + 
			"Precedenza: " + prec +  "\t SLowing Point: " + v.getSlowingPoint() + "\n" + 
			CsString
			);
	}
}