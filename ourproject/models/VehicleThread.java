package se.oru.coordination.coordination_oru.ourproject.models;

public class VehicleThread implements Runnable {
	
	private Vehicle v;
	
	public VehicleThread(Vehicle v){
		this.v = v;
	}
	
	// MAIN ALGORITHM
	public void run() {
		int i = 0;
		String List;
		v.setPathIndex(0);
		//System.out.println(v.getSecForSafety());
		try{
			while(v.getPathIndex() < v.getWholePath().length){		// this will be while true
				v.setMyTimes();
				v.setSpatialEnvelope();
				Thread.sleep(v.getTc());

				//System.out.println(Arrays.toString(v.getMyTimes()));
				//System.out.println(v.getSpatialEnvelope().getPath().length);
				
				// PARTE DI MIC
				List = " ";
				
				v.clearCs();
				for (Vehicle vh : this.v.getNears()){
					v.appendCs(vh);
					List = (List + vh.getID() + " " );
				}
				
				Boolean prec = true;
				for (CriticalSection cs : this.v.getCs()){ //devo averle ordinate
					prec = cs.ComputePrecedences();
					// SPOSTARE IL CALCOLO PUNTO CRITICO DA MOVEVEHICLE A QUI
				}
				v.moveVehicle(prec);
				
				printLog(List, prec);
				i++;
			}
			System.out.println("\n R" + this.v.getID() + " : GOAL RAGGIUNTO" );
		}
		catch (InterruptedException e) {
			System.out.println("Thread interrotto");
		}	
	}

	
	public void printLog(String List, Boolean prec) {
		int nCs;
		String CsString = "";
		if (v.getCs().size() != 0){
			nCs = v.getCs().size();
			for (int i = 0; i < nCs; i++)
				CsString = CsString + (i+1 +"° Sezione Critica" +
				"\t Mia: " + v.getCs().get(i).getTe1Start()+"-"+v.getCs().get(i).getTe1End() +
				"\t Altrui: " + v.getCs().get(i).getTe2Start()+"-"+v.getCs().get(i).getTe2End()) + "\n";
		}
		else {
			nCs = 0;
			CsString = ("Cs " + nCs );
		}
		
		System.out.println("\n R" + this.v.getID() + " : \n" + 
			"Vicini: "  + List + "\n" + 
			"Precedenza: " + prec + "\n" + 
			"Path Index: " 	+ v.getPathIndex() + "\n" +
			"Critical Point: " + v.getCriticalPoint() + "\n" +
			CsString
			);
	}
}

