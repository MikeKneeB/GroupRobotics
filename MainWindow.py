import Tkinter as T
import HSMSim as h

def draw():
    test.iterate()
    w.delete(line)
    w.create_line(200, 0, 200+test.x*100, 200+test.y*100, fill="red")
    top.after(10, draw())

test = h.HorizontalSlidingMassSimulation()

top = T.Tk()

w = T.Canvas(top, width = 400, height = 400)
w.pack()

line = w.create_line(200, 0, 200+test.x*100, 200+test.y*100, fill="red")

top.after(0, draw())
top.mainloop()