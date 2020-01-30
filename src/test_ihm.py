#!/usr/bin/env python
"""
    Copyright 2019:
        Laetitia Lerandy
        Alban Chauvel
        Estelle Arricau

        Projet robotique autonome 2020 DIMR KUKA
        ENSC - ENSEIRB MATMECA 3eme annee option robotique
        code de l'IHM
"""

try:
    # for Python2
    import Tkinter as tk
    import tkFont as tkfont
except ImportError:
    # for Python3
    import tkinter as tk
    from tkinter import font  as tkfont

from PIL import Image, ImageTk



class ihm_kuka(tk.Tk):

    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)

        self.title_font = tkfont.Font(family='Helvetica', size=18, weight="bold", slant="italic")

        # the container is where we'll stack a bunch of frames
        # on top of each other, then the one we want visible
        # will be raised above the others
        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames = {}
        for F in (StartPage, MainPage): # ADD PAGE HERE
            page_name = F.__name__
            frame = F(parent=container, controller=self)
            self.frames[page_name] = frame

            # put all of the pages in the same location;
            # the one on the top of the stacking order
            # will be the one that is visible.
            frame.grid(row=0, column=0, sticky="nsew")

        self.show_frame("StartPage")

    def show_frame(self, page_name):
        '''Show a frame for the given page name'''
        frame = self.frames[page_name]
        frame.tkraise()


class StartPage(tk.Frame):

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        self.controller = controller
        #img = tk.PhotoImage(file="home_img.jpg")
        img = Image.open("home_img.jpg")
        labelWidth = controller.winfo_screenwidth()
        labelHeight = controller.winfo_screenheight()
        maxsize = (labelWidth, labelHeight)
        img.thumbnail(maxsize, Image.ANTIALIAS)
        img_ = ImageTk.PhotoImage(img)

        label = tk.Label(self, image=img_ ,font=controller.title_font)
        label.image = img_
        label.pack()
        button1 = tk.Button(self, text="START THE BUILDING",font=(None,14,'bold'),fg='black',height=3, width=30,
                            command=lambda: controller.show_frame("MainPage"))
        #button2 = tk.Button(self, text="Go to Page Two",command=lambda: controller.show_frame("PageTwo"))
        button1.place(relx=.5, rely=.5, anchor="c")
        #button2.pack()

class  MainPage(tk.Frame):

    #############
    def __init__(self,parent,controller):
        tk.Frame.__init__(self,parent)
        self.parent = parent
        self.column_number=4
        self.layer_number=3
        self.initialize(self.column_number,self.layer_number)
        self.colorate_current_layer(0)


    #############
    def initialize(self,column_number,layer_number):

        self.grid()
        self.bricks = [[0 for x in xrange(column_number+1)] for x in xrange(layer_number)]
        self.order = tk.StringVar()
        self.order.set("Click on a brick to fill the 1st layer")
        begin_row=1
        first_col_num=1
        end_col_num=1
        begin_parity=layer_number%2
        self.color_init="navajo white"
        self.color_current_layer="floral white"
        self.color_current_brick="green"
        self.color_brick_placed="DarkOrange2"

        title = tk.Label(self,text="KUKA BUILDING",font=(None,40,'bold'),anchor="n",bg=self.color_init,pady=15)
        title.grid(row=0,column=0,columnspan=column_number*2+first_col_num+end_col_num,sticky='NSEW')
        self.grid_rowconfigure(0,weight=2)

        for layer in range(layer_number-1,-1,-1):

            tk.Label(self,anchor="center",bg=self.color_init).grid(row=begin_row+layer,column=0,sticky='NSEW')
            self.grid_columnconfigure(0,weight=2)

            if (layer%2)==begin_parity: # 2 small bricks

                self.bricks[layer][0] = tk.Button(self, bg=self.color_init,command=lambda x=abs(layer-(layer_number-1)), y=0: self.select_brick(x,y))
                self.bricks[layer][0].grid(column=first_col_num, row=layer+begin_row,sticky='NSEW')

                cpt=0
                for column in range(1,column_number,1):
                    self.bricks[layer][column] = tk.Button(self, bg=self.color_init,command=lambda x=abs(layer-(layer_number-1)), y=column: self.select_brick(x,y))
                    self.bricks[layer][column].grid(column=first_col_num+column+cpt, row=begin_row+layer,columnspan=2, sticky='NSEW')
                    cpt+=1

                self.bricks[layer][column_number] = tk.Button(self, bg=self.color_init,command=lambda x=abs(layer-(layer_number-1)), y=column_number: self.select_brick(x,y))
                self.bricks[layer][column_number].grid(column=first_col_num+column_number*2-1, row=layer+begin_row,sticky='NSEW')



            else:
                cpt=0
                for column in range(column_number):
                    self.bricks[layer][column] = tk.Button(self, bg=self.color_init,command=lambda x=abs(layer-(layer_number-1)), y=column: self.select_brick(x,y))
                    self.bricks[layer][column].grid(column=first_col_num+column+cpt, row=layer+begin_row,columnspan=2,sticky='NSEW')
                    cpt+=1

            tk.Label(self,anchor="center",bg=self.color_init).grid(row=begin_row+layer,column=first_col_num+column_number*2,sticky='NSEW')
            self.grid_columnconfigure(first_col_num+column_number*2,weight=2)

        for layer in range(begin_row+layer_number-1,-1+begin_row,-1):
            self.grid_rowconfigure(layer,weight=1)
            for column in range(column_number*2):
                self.grid_columnconfigure(column+first_col_num,weight=1)

        label = tk.Label(self,textvariable=self.order,font=(None,20),fg="DarkOrange2",anchor="center",bg=self.color_init,pady=15)
        label.grid(row=begin_row+layer_number,column=0,columnspan=column_number*2+first_col_num+end_col_num,sticky='NSEW')
        self.grid_rowconfigure(begin_row+layer_number,weight=1)


    #############
    def colorate_current_layer(self,layer):

        for y in range(self.column_number+(layer%2)):
            self.bricks[abs(layer-(self.layer_number-1))][y].config(bg=self.color_current_layer)


    #############
    def select_brick(self, layer, column):
        print(layer,column)
        self.bricks[abs(layer-(self.layer_number-1))][column].config(bg=self.color_brick_placed)

    #############
    def rgb2hex(self, r, g, b):
        return "#%02x%02x%02x" % (r, g, b)

# class PageOne(tk.Frame):
#
#     def __init__(self, parent, controller):
#         tk.Frame.__init__(self, parent)
#         self.controller = controller
#         label = tk.Label(self, text="This is page 1", font=controller.title_font)
#         label.pack(side="top", fill="x", pady=10)
#         button = tk.Button(self, text="Go to the start page",
#                            command=lambda: controller.show_frame("StartPage"))
#         button.pack()


# class PageTwo(tk.Frame):
#
#     def __init__(self, parent, controller):
#         tk.Frame.__init__(self, parent)
#         self.controller = controller
#         label = tk.Label(self, text="This is page 2", font=controller.title_font)
#         label.pack(side="top", fill="x", pady=10)
#         button = tk.Button(self, text="Go to the start page",
#                            command=lambda: controller.show_frame("StartPage"))
#         button.pack()


if __name__ == "__main__":
    app = ihm_kuka()
    app.title("DIMR KUKA")
    app.geometry('800x500')
    app.mainloop()
