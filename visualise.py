import multiprocessing
import time

def visualise_process(queue: multiprocessing.Queue):
    """
    Function to be run in the visualiser process.
    It continuously reads data from the provided queue.
    """
    print("Visualise process started, waiting for data...")
    while True:
        try:
            # Get data from the queue. block=True means it will wait until data is available.
            # timeout can be used to prevent indefinite blocking, but for this example, we'll block.
            data = queue.get()
            if data == "STOP":
                print("Visualise: Received 'STOP' signal. Terminating.")
                break
            print(f"Visualise: Received data: '{data}'")
            # Simulate some processing time
            time.sleep(0.5)
        except Exception as e:
            print(f"Visualise: An error occurred: {e}")
            break
    print("Visualise process finished.")
