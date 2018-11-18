"""
Sample Python/Pygame Programs
Simpson College Computer Science
http://programarcadegames.com/
http://simpson.edu/computer-science/

Show everything we can pull off the joystick
"""
import pygame

# Define some colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)


class TextPrint(object):
    """
    This is a simple class that will help us print to the screen
    It has nothing to do with the joysticks, just outputting the
    information.
    """
    def __init__(self):
        """ Constructor """
        self.reset()
        self.x_pos = 10
        self.y_pos = 10
        self.font = pygame.font.Font(None, 20)

    def pprint(self, my_screen, text_string):
        """ Draw text onto the screen. """
        text_bitmap = self.font.render(text_string, True, BLACK)
        my_screen.blit(text_bitmap, [self.x_pos, self.y_pos])
        self.y_pos += self.line_height

    def reset(self):
        """ Reset text to the top of the screen. """
        self.x_pos = 10
        self.y_pos = 10
        self.line_height = 15

    def indent(self):
        """ Indent the next line of text """
        self.x_pos += 10

    def unindent(self):
        """ Unindent the next line of text """
        self.x_pos -= 10


pygame.init()

# Set the width and height of the screen [width,height]
size = [500, 700]
screen = pygame.display.set_mode(size)

pygame.display.set_caption("My Game")

# Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

# Initialize the joysticks
pygame.joystick.init()

# Get ready to print
textPrint = TextPrint()

# -------- Main Program Loop -----------
while not done:
    # EVENT PROCESSING STEP
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True

        # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN
        # JOYBUTTONUP JOYHATMOTION
        if event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
        if event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")

    # DRAWING STEP
    # First, clear the screen to white. Don't put other drawing commands
    # above this, or they will be erased with this command.
    screen.fill(WHITE)
    textPrint.reset()

    # Get count of joysticks
    joystick_count = pygame.joystick.get_count()

    textPrint.pprint(screen, "Number of joysticks: {}".format(joystick_count))
    textPrint.indent()

    # For each joystick:
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()

        textPrint.pprint(screen, "Joystick {}".format(i))
        textPrint.indent()

        # Get the name from the OS for the controller/joystick
        name = joystick.get_name()
        textPrint.pprint(screen, "Joystick name: {}".format(name))

        # Usually axis run in pairs, up/down for one, and left/right for
        # the other.
        axes = joystick.get_numaxes()
        textPrint.pprint(screen, "Number of axes: {}".format(axes))
        textPrint.indent()

        #for i in range(axes):
        #    axis = joystick.get_axis(i)
        #    textPrint.pprint(screen, "Axis {} value: {:>6.3f}".format(i, axis))
        axis_0 = joystick.get_axis(0)
        textPrint.pprint(screen, "Axis {} value: {:>6.3f}".format('L-:noting', axis_0))

        axis_1 = joystick.get_axis(1)
        textPrint.pprint(screen, "Axis {} value: {:>6.3f}".format('L|:noting', axis_1))

        axis_2 = joystick.get_axis(2)
        textPrint.pprint(screen, "Axis {} value: {:>6.3f}".format('Lt:brake', axis_2))

        axis_3 = joystick.get_axis(3)
        textPrint.pprint(screen, "Axis {} value: {:>6.3f}".format('G-:steer', axis_3))

        axis_4 = joystick.get_axis(4)
        textPrint.pprint(screen, "Axis {} value: {:>6.3f}".format('G|:noting', axis_4))

        axis_5 = joystick.get_axis(5)
        textPrint.pprint(screen, "Axis {} value: {:>6.3f}".format('Gt:accel', axis_5))

        textPrint.unindent()

        buttons = joystick.get_numbuttons()
        textPrint.pprint(screen, "Number of buttons: {}".format(buttons))
        textPrint.indent()

        #for i in range(buttons):
        #    button = joystick.get_button(i)
        #    textPrint.pprint(screen, "Button {:>2} value: {}".format(i, button))
        button_0 = joystick.get_button(0)
        textPrint.pprint(screen, "Button {:>2} value: {}".format('A:nothing', button_0))
        button_1 = joystick.get_button(1)
        textPrint.pprint(screen, "Button {:>2} value: {}".format('B:nothing', button_1))
        button_2 = joystick.get_button(2)
        textPrint.pprint(screen, "Button {:>2} value: {}".format('X:nothing', button_2))
        button_3 = joystick.get_button(3)
        textPrint.pprint(screen, "Button {:>2} value: {}".format('Y:nothing', button_3))
        button_4 = joystick.get_button(4)
        textPrint.pprint(screen, "Button {:>2} value: {}".format('LB:backward', button_4))
        button_5 = joystick.get_button(5)
        textPrint.pprint(screen, "Button {:>2} value: {}".format('RB:forward', button_5))
        button_6 = joystick.get_button(6)
        textPrint.pprint(screen, "Button {:>2} value: {}".format('BACK:nothing', button_6))
        button_7 = joystick.get_button(7)
        textPrint.pprint(screen, "Button {:>2} value: {}".format('START:nothing', button_7))
        button_8 = joystick.get_button(8)
        textPrint.pprint(screen, "Button {:>2} value: {}".format(8, button_8))
        button_9 = joystick.get_button(9)
        textPrint.pprint(screen, "Button {:>2} value: {}".format(9, button_9))
        button_10 = joystick.get_button(10)
        textPrint.pprint(screen, "Button {:>2} value: {}".format(10, button_10))
        textPrint.unindent()

        # Hat switch. All or nothing for direction, not like joysticks.
        # Value comes back in an array.
        # (right/left,up/down)
        hats = joystick.get_numhats()
        textPrint.pprint(screen, "Number of hats: {}".format(hats))
        textPrint.indent()

        for i in range(hats):
            hat = joystick.get_hat(i)
            textPrint.pprint(screen, "Hat {} value: {}".format(i, str(hat)))
        textPrint.unindent()

        textPrint.unindent()

    # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT

    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()

    # Limit to 60 frames per second
    clock.tick(60)

# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
pygame.quit()
