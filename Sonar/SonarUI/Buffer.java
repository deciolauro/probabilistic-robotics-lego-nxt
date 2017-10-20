class Buffer
{
    private Reading reading;
    private int[] readings;
    private boolean available = false;
	private boolean completedFullScan = false;

    Buffer()
    {
        reading = new Reading();
        readings = new int[91]; 
    }   

	public synchronized boolean wasModified()
	{
		return available;
	}

	public synchronized boolean completedScan()
	{
		return completedFullScan;
	}

    public synchronized Reading get()
    {
        while (available == false)
        {
            try
            {
                wait();
            }
            catch (InterruptedException e) {}
        }
        available = false;
        notifyAll();
        return reading;
    }

	public synchronized int read(int i)
    {
		return readings[i];
    }

	public synchronized int[] read()
    {
		return readings;
    }

	public synchronized void put(int angle, int dist)
    {
        while(available == true)
        {
			try
			{
				wait();
			}
			catch (InterruptedException e) {}
		}
        reading.set(angle, dist);
        readings[angle/2] = dist;
        available = true;
		if(angle==180)
		{
			completedFullScan = true;
		}
		else if(completedFullScan)
		{
			completedFullScan = false;
		}

		notifyAll();
    }
}
