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
from Domain.wall import Wall

class Ihm(tk.Tk):
    def __init__(self, feeders):
        tk.Tk.__init__(self)
        self.feeders = feeders
        self.wall = None
        self.title_font = tkfont.Font(family='Helvetica', size=18, weight="bold", slant="italic")

        # the container is where we'll stack a bunch of frames
        # on top of each other, then the one we want visible
        # will be raised above the others
        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)
        self.column_number=4
        self.layer_number=3

        self.frames = {}
        for F in (StartPage, MainPage, Parameter): # ADD PAGE HERE
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
        print ("testshowframes",self.layer_number,self.column_number)
        frame = self.frames[page_name]
        frame.tkraise()

#=======================
class StartPage(tk.Frame):

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        self.controller = controller
        #img = tk.PhotoImage(file="home_img.jpg")
        img = Image.open("Images/home_img.jpg")
        labelWidth = controller.winfo_screenwidth()
        labelHeight = controller.winfo_screenheight()
        maxsize = (labelWidth, labelHeight)
        img.thumbnail(maxsize, Image.ANTIALIAS)
        img_ = ImageTk.PhotoImage(img)

        img_param = Image.open("Images/parameter.png")
        img_param.thumbnail((50,50), Image.ANTIALIAS)
        img_p = ImageTk.PhotoImage(img_param)

        label = tk.Label(self, image=img_ ,font=controller.title_font)
        label.image = img_
        label.pack()
        button1 = tk.Button(self, text="START THE BUILDING",font=(None,14,'bold'),fg='black',height=3, width=30,
                            command=self.startBuild)
        button2 = tk.Button(self,font=(None,10,'bold'),image = img_p,command=lambda: controller.show_frame("Parameter"))
        button2.image=img_p
        button1.place(relx=.5, rely=.5, anchor="c")
        button2.place(relx=1.,anchor="ne",bordermode="outside")

    def startBuild(self):
        self.controller.frames["MainPage"].initialize()
        self.controller.show_frame("MainPage")
        self.controller.wall = Wall(self.controller.feeders, self.controller.layer_number, self.controller.column_number)
        self.controller.wall.to_string()


#=======================
class  MainPage(tk.Frame):

    #############
    def __init__(self,parent,controller):
        tk.Frame.__init__(self,parent)
        self.parent = parent
        self.controller=controller
        print("testputain",controller.column_number,controller.layer_number)
        self.column_number=self.controller.column_number
        self.layer_number=self.controller.layer_number


    #############
    def initialize(self):
        self.column_number=self.controller.column_number
        self.layer_number=self.controller.layer_number
        self.grid()
        self.bricks = [[0 for x in xrange(self.column_number+1)] for x in xrange(self.layer_number)]
        self.arrows = [[0 for x in xrange(2)] for x in xrange(self.layer_number)]
        self.order = tk.StringVar()
        self.order.set("Click on a brick to fill the 1st layer")
        begin_row=1
        first_col_num=1
        end_col_num=1
        begin_parity=self.layer_number%2
        self.color_init="navajo white"
        self.color_current_layer="floral white"
        self.color_current_brick="green"
        self.color_brick_placed="DarkOrange2"
        self.current_layer=0

        title = tk.Label(self,text="KUKA BUILDING",font=(None,40,'bold'),anchor="n",bg=self.color_init,pady=15)
        title.grid(row=0,column=0,columnspan=self.column_number*2+first_col_num+end_col_num,sticky='NSEW')
        self.grid_rowconfigure(0,weight=2)

        for layer in range(self.layer_number-1,-1,-1):

            self.arrows[layer][0]=tk.Label(self,anchor="center",text ="" ,fg="DarkOrange2",font=(None,20,'bold'),bg=self.color_init)
            self.arrows[layer][0].grid(row=begin_row+layer,column=0,sticky='NSEW')
            self.grid_columnconfigure(0,weight=2)

            if (layer%2)==begin_parity: # 2 small bricks

                self.bricks[layer][0] = tk.Button(self, bg=self.color_init,command=lambda x=abs(layer-(self.layer_number-1)), y=0: self.select_brick(x,y))
                self.bricks[layer][0].grid(column=first_col_num, row=layer+begin_row,sticky='NSEW')

                cpt=0
                for column in range(1,self.column_number,1):
                    self.bricks[layer][column] = tk.Button(self, bg=self.color_init,command=lambda x=abs(layer-(self.layer_number-1)), y=column: self.select_brick(x,y))
                    self.bricks[layer][column].grid(column=first_col_num+column+cpt, row=begin_row+layer,columnspan=2, sticky='NSEW')
                    cpt+=1

                self.bricks[layer][self.column_number] = tk.Button(self, bg=self.color_init,command=lambda x=abs(layer-(self.layer_number-1)), y=self.column_number: self.select_brick(x,y))
                self.bricks[layer][self.column_number].grid(column=first_col_num+self.column_number*2-1, row=layer+begin_row,sticky='NSEW')

            else:
                cpt=0
                for column in range(self.column_number):
                    self.bricks[layer][column] = tk.Button(self, bg=self.color_init,command=lambda x=abs(layer-(self.layer_number-1)), y=column: self.select_brick(x,y))
                    self.bricks[layer][column].grid(column=first_col_num+column+cpt, row=layer+begin_row,columnspan=2,sticky='NSEW')
                    cpt+=1

            self.arrows[layer][1]=tk.Label(self,text ="" ,fg="DarkOrange2",font=(None,20,'bold'),anchor="center",bg=self.color_init)
            self.arrows[layer][1].grid(row=begin_row+layer,column=first_col_num+self.column_number*2,sticky='NSEW')
            self.grid_columnconfigure(first_col_num+self.column_number*2,weight=2)

        for layer in range(begin_row+self.layer_number-1,-1+begin_row,-1):
            self.grid_rowconfigure(layer,weight=1)
            for column in range(self.column_number*2):
                self.grid_columnconfigure(column+first_col_num,weight=1)

        label = tk.Label(self,textvariable=self.order,font=(None,20),fg="DarkOrange2",anchor="center",bg=self.color_init,pady=15)
        label.grid(row=begin_row+self.layer_number,column=0,columnspan=self.column_number*2+first_col_num+end_col_num,sticky='NSEW')
        self.grid_rowconfigure(begin_row+self.layer_number,weight=1)

        img_bin = Image.open("Images/bin2.png")
        img_bin.thumbnail((70,70), Image.ANTIALIAS)
        img_b = ImageTk.PhotoImage(img_bin)
        button_destroy = tk.Button(self,font=(None,10,'bold'),image = img_b, bg=self.color_init, command=self.destroy)
        button_destroy.image=img_b
        button_destroy.place(relx=1.,anchor="ne",bordermode="outside")
        self.update_arrows(self.current_layer)
        self.colorate_current_layer(0)



    #############
    def update_arrows(self,current_layer):
        for layer in range(self.layer_number):
            if layer==current_layer:
                self.arrows[abs(layer-(self.layer_number-1))][0].config(text="----->")
                self.arrows[abs(layer-(self.layer_number-1))][1].config(text="<-----")
            else:
                self.arrows[abs(layer-(self.layer_number-1))][0].config(text="")
                self.arrows[abs(layer-(self.layer_number-1))][1].config(text="")

    #############
    def colorate_current_layer(self,layer):

        for y in range(self.column_number+(layer%2)):
            self.bricks[abs(layer-(self.layer_number-1))][y].config(bg=self.color_current_layer)


    #############
    def select_brick(self, layer, column):
        print(layer,column)
        if layer == self.current_layer:
            self.bricks[abs(layer-(self.layer_number-1))][column].config(bg=self.color_brick_placed)

    #############
    def destroy(self):
        print("destroying the wall")
        for layer in range(self.layer_number):
            for column in range(self.column_number+1):
                if not self.bricks[layer][column]==0:
                    self.bricks[layer][column].config(bg=self.color_init)

        self.colorate_current_layer(0)
        print("destroy done")

    #############
    def rgb2hex(self, r, g, b):
        return "#%02x%02x%02x" % (r, g, b)



#=======================
class Parameter(tk.Frame):

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        self.controller = controller
        self.grid()
        self.title = tk.Label(self,text="Parameters",font=(None,40,'bold'),anchor="n",pady=15)
        self.title.grid(row=0,column=0,columnspan=2,sticky='NSEW')
        self.grid_rowconfigure(0,weight=2)
        self.label_layer = tk.Label(self,text="Number of layer : ",font=(None,20),anchor="n",pady=15)
        self.label_layer.grid(row=1,column=0,sticky='NSEW')
        self.grid_columnconfigure(0,weight=1)
        self.grid_rowconfigure(1,weight=1)
        self.label_brick = tk.Label(self,text="Number of column : ",font=(None,20),anchor="n",pady=15)
        self.label_brick.grid(row=2,column=0,sticky='NSEW')
        self.grid_rowconfigure(2,weight=1)
        self.layer_slider = tk.Scale(self,from_=1, to=3,bd=3, sliderlength=100,orient="horizontal")
        self.layer_slider.grid(row=1,column=1,sticky='NSEW',padx=50)
        self.layer_slider.set(3)
        self.grid_columnconfigure(1,weight=2)
        self.brick_slider = tk.Scale(self,from_=1, to=4,bd=3,sliderlength=80,orient="horizontal")
        self.brick_slider.grid(row=2,column=1,sticky='NSEW',padx=50)
        self.brick_slider.set(4)

        self.button = tk.Button(self, text="OK",font=(None,17),bg="DarkSeaGreen1",
                           command=self.show_values)
        self.grid_rowconfigure(3,weight=1)
        self.button.grid(row=3,column=0,columnspan=2,sticky='NSEW')

    def show_values(self):
        print (self.layer_slider.get(),self.brick_slider.get())
        self.controller.layer_number=self.layer_slider.get()
        self.controller.column_number=self.brick_slider.get()
        self.controller.show_frame("StartPage")

##=======================
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
