import pygame as pg

h = 600
v = 600

# size of arena in metres
arena_x = 3
arena_y = 3

# Setup for pygame window
pg.init()
pg.display.set_caption('Navigation Map')
window = pg.display.set_mode((h, v))


# Setup background surface
background = pg.Surface((h, v))
background.fill(pg.Color('#000000'))

# Setup robot representation
robot = pg.image.load('media/drone.png').convert_alpha()

# Setup fruit representation 
apple = pg.image.load('media/apple.png').convert_alpha()
pear = pg.image.load('media/pear.png').convert_alpha()

# Draw horizontal lines
for i in range(1,3):
    horiz = v / arena_y
    pg.draw.line(background, (255,255,255), (0, i * horiz),(h, i * horiz))


# Draw vertical lines
for i in range(1,3):
    vert = h / arena_x
    pg.draw.line(background, (255,255,255), (i * vert, 0),(i * vert, v))


running = True
while running:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False
            break

        # Detect mouse click
        elif event.type == pg.MOUSEBUTTONDOWN:
            (x,y) = pg.mouse.get_pos()
            x_metre = x * arena_x / h
            y_metre = y * arena_y / v
            print(f"Mouse is at ({x_metre},{y_metre})")

    # Display static elements here
    # TODO: show location of markers
    # TODO: show location of fruit

    # Display dynamic elements here
    # TODO: show location of robot


    window.blit(background, (0, 0))
    window.blit(robot, (100,100))
    window.blit(apple, (200,200))
    window.blit(pear, (150,150))
    pg.display.update()
pg.quit()

