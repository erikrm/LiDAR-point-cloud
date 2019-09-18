from tkinter import *
from tkinter.filedialog import askopenfilename

def get_input_file_from_dialog(title,file_path,file_type):
    Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing
    filename = askopenfilename(initialdir=file_path, title = title, filetypes = ((file_type + " files" , "*." + file_type),("all files" , "*.*"))) # show an "Open" dialog box and return the path to the selected file
    return file_name


root = Tk()

topFrame = Frame(root)
topFrame.pack()

bottomFrame = Frame(root)
bottomFrame.pack(side=BOTTOM)

buttonChooseInputPcap = Button(topFrame, text="Choose PCAP file", command=)
buttonChooseInputIns = Button(topFrame, text="Choose INS file")
buttonChooseInputPcap.pack(side=LEFT)
buttonChooseInputIns.pack(side=LEFT)


theLabel = Label(bottomFrame, text="Wireshark")
theLabel.pack(side=RIGHT)

input("wait")

