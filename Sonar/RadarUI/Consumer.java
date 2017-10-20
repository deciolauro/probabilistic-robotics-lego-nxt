// DEPRECATED ---- NOT IN USE -----
class Consumer extends Thread
{
	private Buffer buffer;

	public Consumer(Buffer b)
	{
		this.buffer = b;
	}

	public void run()
	{
		Reading reading;
		while(true)
		{
			reading = this.buffer.get();
      		System.out.println("Consumer: " + reading.getAngle() + ", " + this.buffer.read(reading.getAngle()));
		}
	}
}
