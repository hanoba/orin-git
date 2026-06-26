import queue
from time import perf_counter_ns

# Gemeinsamer FIFO-Puffer (non-blocking Trace)
MAX_SIZE = 1000

class Trace:
    def __init__(self, traceStart=None):
        self.trace_queue = queue.Queue(maxsize=MAX_SIZE)
        self.traceQueueFull = False
        self.traceStart = perf_counter_ns() if traceStart==None else traceStart

    # Nicht-blockierende Trace-Funktion."""
    def Put(self, msg: str):
        if self.traceQueueFull: return
        if self.trace_queue.full(): 
            self.traceQueueFull = True
            return
        t = perf_counter_ns() - self.traceStart
        self.trace_queue.put_nowait(f"{t*1e-6:.3f} {msg}")

    def Print(self):
        while not self.trace_queue.empty():
            print(self.trace_queue.get())

    def Save(self, filename="trace.log"):
        # "w" überschreibt die Datei jedes Mal neu. 
        # Wenn du an eine bestehende Datei anhängen willst, nutze "a" (append).
        with open(filename, "w", encoding="utf-8") as file:
            while not self.trace_queue.empty():
                item = self.trace_queue.get()
                # '\n' sorgt für den Zeilenumbruch, da file.write() das nicht automatisch macht
                file.write(f"{item}\n")