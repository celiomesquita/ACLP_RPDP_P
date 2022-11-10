import multiprocessing as mp
import time
   
def queuePut(queue, events):

    for i, event in enumerate(events):

        if event == "eod":
            events[i] = "End of Day"

        print(f"Enqueued: {events[i]}")
        # time.sleep(0.5)
        queue.put(events[i])
  
def queueGet(queue, events):

    evts = []

    for _ in events:

        event = queue.get()

        if event == "eod":
            event = "End of Day"

        evts.append(event)
        print(f"Dequeued: {event}")

    events = evts
       

def run():
    events = ["get up", "brush your teeth", "shower", "work", "eod"]

    queue = mp.Queue()
    process_3 = mp.Process(target=queuePut, args=(queue, events))
    process_4 = mp.Process(target=queueGet, args=(queue, events))  

    process_3.start()
    process_4.start()

    process_3.join()

    process_4.join()

    print(events)


if __name__ == "__main__":
    run()
 

  