'''
Inverse Kinematics Simulator
Developed in 2024 by Arshan Soltanzadeh, La Canada High School class of 2027
Implements custom algorithm to find joint angles in order to move end defector to desired point
'''

#import pygame for graphics and math for trig
import pygame
import math
import sys
from pygame.locals import *
from unicodedata import *

#init pygame and setup screen + colors, and add font
pygame.init()
WINDOW_LENGTH = 1000
WINDOW_HEIGHT = 800

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 150, 0)
BLUE = (0, 0, 255)

basicFont = pygame.font.SysFont("Arial", 48)

windowSurface = pygame.display.set_mode((WINDOW_LENGTH, WINDOW_HEIGHT))
pygame.display.set_caption("Inverse Kinematics Simulator") 


#pythagorean theorem
def distance(x1, y1, x2, y2):
    dx = x2-x1
    dy = y2-y1
    return math.sqrt(dx*dx + dy*dy)


#cartesian to pygame coordinates and vice versa
def coordinatesToPygame(x, y):
    return ((WINDOW_LENGTH / 2)+x, (WINDOW_HEIGHT/2)-y)

def coordinatesToCartesian(x, y):
    return (x-(WINDOW_LENGTH/2), -y + (WINDOW_HEIGHT/2))



#main inverse kinematics function
def inverseKinematics(arm1, arm2, x, y):
    endx = coordinatesToCartesian(x, y)[0]
    endy = coordinatesToCartesian(x, y)[1]

    #length of straight line from initial to final point
    #c = distance(arm1.x1, arm1.y1, arm2.x1, arm2.y1)
    c = distance(arm1.x1, arm1.y1, endx, endy)

    #if c is greater than both arm lengths, then set end x and end y to the furthest possible distances such that the hypotenuse of the triangle created with endx and endy to not surpass the sum of both arm lengths
    if c >= (arm1.length + arm2.length):
        endx = (arm1.length + arm2.length) * math.cos(math.atan2((arm1.y1 - endy), (arm1.x1 - endx)))
        endy = (arm1.length + arm2.length) * math.sin(math.atan2((arm1.y1 - endy), (arm1.x1 - endx)))
    
    #uncomment this code to limit the arm movement to the first quadrant
    #if endx <= 0 or endy <= 0:
        #endx = arm1.length / 2
        #endy = arm1.length / 2

    #set C again
    c = distance(arm1.x1, arm1.y1, endx, endy)

    beta = math.degrees(math.atan2(endy, endx))

    try:
        alpha = math.degrees(math.acos(((-(arm2.length**2) + arm1.length**2 + c**2)/(2*arm1.length*c))))

    except ValueError:
        alpha = 3

    try:
        phi = math.degrees(math.acos(((-(c**2) + arm1.length**2 + arm2.length**2) / (2*arm1.length*arm2.length))))
   
    except ValueError:
        return

    arm1.angle = beta + alpha

    #to render second arm correctly in pygame, calculate this (in real life, the second arm angle would just be phi):
    arm2.angle = phi - 180 + alpha + beta

    #the true realistic angle of arm2 is rangle, or just phi
    arm2.rangle = phi
    


#arm class
arms = []

class Arm:
    def __init__(self, x, y, length, thickness, color, isPrimary, angle=0):
        self.x1 = x
        self.y1 = y
        self.length = length
        self.thickness = thickness
        self.color = color
        self.angle = angle
        self.rangle = 0

        self.x2 = self.x1 + (self.length * math.cos(math.radians(self.angle)))
        self.y2 = self.y1 + (self.length * math.sin(math.radians(self.angle)))
        self.x3 = self.x2 - (self.thickness * math.cos(math.radians(90 - self.angle)))
        self.y3 = self.y2 + (self.thickness * math.sin(math.radians(90 - self.angle)))
        self.x4 = self.x1 - self.thickness * math.cos(math.radians(90-self.angle))
        self.y4 = self.y1 + self.thickness * math.sin(math.radians(90-self.angle))

        arms.append(self)

        #boolean to determine if arm is primary connected to initial point, or if it is second in the chain and its end is the final point
        self.isPrimary = isPrimary

    def update(self, x, y):
        if self.isPrimary:
            inverseKinematics(self, arms[1], x, y)

        else:
            self.x1 = arms[0].x2
            self.y1 = arms[0].y2
        
        self.x2 = self.x1 + (self.length * math.cos(math.radians(self.angle)))
        self.y2 = self.y1 + (self.length * math.sin(math.radians(self.angle)))
        self.x3 = self.x2 - (self.thickness * math.cos(math.radians(90 - self.angle)))
        self.y3 = self.y2 + (self.thickness * math.sin(math.radians(90 - self.angle)))
        self.x4 = self.x1 - self.thickness * math.cos(math.radians(90 - self.angle))
        self.y4 = self.y1 + self.thickness * math.sin(math.radians(90 - self.angle))
        

    def render(self):
        pygame.draw.polygon(windowSurface, self.color, (coordinatesToPygame(self.x1, self.y1), coordinatesToPygame(self.x2, self.y2), coordinatesToPygame(self.x3, self.y3), coordinatesToPygame(self.x4, self.y4)))


#create arms
arm1 = Arm(0, 0, 200, 30, "WHITE", True)
arm2 = Arm(arm1.x2, arm1.y2, 150, 30, "BLUE", False)


x = (arm1.length + arm2.length) * math.cos(math.degrees(80))
y = (arm1.length + arm2.length) * math.sin(math.degrees(80))



#create input box class
inputBoxes = []

FONT = pygame.font.Font(None, 32)


class InputBox:

    def __init__(self, x, y, w, h, text=''):
        self.rect = pygame.Rect(x, y, w, h)
        self.color_inactive = RED
        self.color_active = BLUE
        self.color = self.color_inactive
        self.text = text
        self.txt_surface = FONT.render(text, True, self.color)
        self.active = False
        inputBoxes.append(self)

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            # If the user clicked on the input_box rect.
            if self.rect.collidepoint(event.pos):
                # Toggle the active variable.
                self.active = not self.active
            else:
                self.active = False
            # Change the current color of the input box.
            self.color = self.color_active if self.active else self.color_inactive
        if event.type == pygame.KEYDOWN:
            if self.active:
                if event.key == pygame.K_BACKSPACE:
                    self.text = self.text[:-1]
                else:
                    self.text += event.unicode.encode()
                # Re-render the text.
                self.txt_surface = FONT.render(self.text, True, self.color)

    def update(self):
        # Resize the box if the text is too long.
        width = max(200, self.txt_surface.get_width()+10)
        self.rect.w = width

    def draw(self, screen):
        # Blit the text.
        screen.blit(self.txt_surface, (self.rect.x+5, self.rect.y+5))
        # Blit the rect.
        pygame.draw.rect(screen, self.color, self.rect, 2)



#create text boxes and text on screen
armLengthInputBox1 = InputBox(250, 5, 100, 30, str(arm1.length).encode("utf-8"))
armLengthInputBox2 = InputBox(215, 58, 100, 30, str(arm2.length).encode("utf-8"))

arm1LengthInputLabel = FONT.render("Bottom Arm Length: ", True, WHITE)
arm2LengthInputLabel = FONT.render("Top Arm Length: ", True, WHITE)



#main loop
while True:
    #handle events
    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit("\n\n*** Simulation Exited *** \n\n")

        #handle input boxes events
        for box in inputBoxes:
            box.handle_event(event)
        

    #clear screen
    windowSurface.fill(BLACK)

    x, y = pygame.mouse.get_pos()

    for box in inputBoxes:
        box.update()

    try:
        arm1.length = float(armLengthInputBox1.text)
        if arm1.length == 0:
            arm1.length = 25
    except:
        pass
    try:
        arm2.length = float(armLengthInputBox2.text)
        if arm2.length == 0:
            arm2.length = 25
    except:
        pass

    for arm in arms:
        arm.update(x, y)

    arm1.render()
    arm2.render()

    for arm in arms:
        arm.render()
    
    for box in inputBoxes:
        box.draw(windowSurface)

    windowSurface.blit(arm1LengthInputLabel, (10, 10))
    windowSurface.blit(arm2LengthInputLabel, (10, 60))

    windowSurface.blit(FONT.render("Bottom Arm Angle: {} deg".format(int(arm1.angle)), True, WHITE), (WINDOW_LENGTH/2, 10))
    windowSurface.blit(FONT.render("Top Arm Angle: {} deg".format(int(arm2.rangle)), True, WHITE), (WINDOW_LENGTH/2, 60))

    #update display
    pygame.display.update()