import queue
from time import perf_counter_ns

# Gemeinsamer FIFO-Puffer (non-blocking Trace)
MAX_SIZE = 100
trace_queue = queue.Queue(maxsize=MAX_SIZE)

traceQueueFull = False
traceStart = perf_counter_ns()

# Nicht-blockierende Trace-Funktion."""
def Trace(msg: str):
    global traceQueueFull
    if traceQueueFull: return
    if trace_queue.full(): 
        traceQueueFull = True
        return
    t = perf_counter_ns() - traceStart
    trace_queue.put_nowait(f"{t*1e-6:.3f} {msg}")

def PrintTrace():
    while not trace_queue.empty():
        print(trace_queue.get())
