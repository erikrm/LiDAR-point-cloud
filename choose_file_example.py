from tkinter import Tk
from tkinter.filedialog import askopenfilename

Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing
filename = askopenfilename(initialdir='./', title = "Select flight", filetypes = (("pcap files","*.pcap"),("all files","*.*"))) # show an "Open" dialog box and return the path to the selected file
print(filename)