import multiprocessing as mp
  
def connSend(conn, events):
    for event in events:
        conn.send(event)
        print(f"Event Sent: {event}")
  
def connRecv(conn):
    while True:
        event = conn.recv()
        if event == "eod": 
            print("Event Received: End of Day")    
            return
        print(f"Event Received: {event}")
  
def queuePut(queue, events):
    for event in events:
        queue.put(event)
        print(f"Event enqueued: {event}")
  
def queueGet(queue):
    while True:
        event = queue.get()
        if event == "eod": 
            print("Event dequeued: End of Day")    
            return
        print(f"Event dequeued: {event}")  

def lockFunction(lock, p):
    lock.acquire()
    print("CRITICAL SECTION")
    print(f"{p} Only One Process has to access at a given time")
    lock.release()

def run():
    events = ["get up", "brush your teeth", "shower", "work", "eod"]

    conn1, conn2 = mp.Pipe()
    process_1 = mp.Process(target=connSend, args=(conn1, events))
    process_2 = mp.Process(target=connRecv, args=(conn2,))

    queue = mp.Queue()
    process_3 = mp.Process(target=queuePut, args=(queue, events))
    process_4 = mp.Process(target=queueGet, args=(queue,))  

    lock = mp.Lock()
    process_5 = mp.Process(target=lockFunction, args=(lock,5))
    process_6 = mp.Process(target=lockFunction, args=(lock,6))

    process_1.start()
    process_2.start()
    process_3.start()
    process_4.start()
    process_5.start()
    process_6.start()

    process_1.join()
    process_2.join()
    process_3.join()
    process_4.join()
    process_5.join()
    process_6.join()


if __name__ == "__main__":
    run()
 

  