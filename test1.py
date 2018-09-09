import turtle

wn = turtle.Screen()
wn.bgcolor("lightgreen")        # set the window background color

tess = turtle.Turtle()
tess.color("blue")              # make tess blue
tess.pensize(3)                 # set the width of her pen

john = turtle.Turtle()
john.color("red")              # make john red
john.pensize(25)                 # set the width of her pen
john.penup()
john.goto(0, 0)
john.pendown()
john.dot()

i = 0
while i < 1000:
    tess.forward(i+50)
    tess.left(i+120)
    tess.forward(i+50)
    i+= 1

wn.exitonclick()                # wait for a user click on the canvas
