package se.oru.coordination.coordination_oru.distributed.models;

import se.oru.coordination.coordination_oru.distributed.algorithms.PrecedencesFounder;

public class CriticalSection implements Comparable<CriticalSection> {
	private int te1Start = -1;
	private int te2Start = -1;
	private int te1End = -1;
	private int te2End = -1;
	private Vehicle v1;
	private RobotReport v2;
	private boolean csTruncated;
	private boolean precedenza;

	public CriticalSection(Vehicle v1, RobotReport v2, int te1Start, int te2Start, int te1End, int te2End,
			boolean csTruncated) {
		this.te1Start = te1Start;
		this.te2Start = te2Start;
		this.te1End = te1End;
		this.te2End = te2End;
		this.v1 = v1;
		this.v2 = v2;
		this.csTruncated = csTruncated;
	}

	public boolean isPrecedenza() {
		return precedenza;
	}

	public void setPrecedenza(boolean precedenza) {
		this.precedenza = precedenza;
	}

	public boolean isCsTruncated() {
		return csTruncated;
	}


	public int getTe1Start() {
		return te1Start;
	}
	public void setTe1Start(int te1Start) {
		this.te1Start = te1Start;
	}
	public int getTe2Start() {
		return te2Start;
	}
	public void setTe2Start(int te2Start) {
		this.te2Start = te2Start;
	}
	public int getTe1End() {
		return te1End;
	}
	public void setTe1End(int te1End) {
		this.te1End = te1End;
	}
	public int getTe2End() {
		return te2End;
	}
	public void setTe2End(int te2End) {
		this.te2End = te2End;
	}
	public Vehicle getVehicle1() {
		return v1;
	}
	public void setVehicle1(Vehicle v1) {
		this.v1 = v1;
	}
	public RobotReport getVehicle2() {
		return v2;
	}
	public void setVehicle2(RobotReport v2) {
		this.v2 = v2;
	}
	
	@Override
    public int compareTo(CriticalSection cs) {
		int compare = this.getTe1Start() - cs.getTe1Start();
		if (compare == 0) compare = 1;
        return compare;
    }
	


}
