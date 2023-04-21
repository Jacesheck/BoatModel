import pygame
import numpy as np
import time

pygame.init()
screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
running = True

class Boat:
    def __init__(self, x, y, w):
        self.w = w
        self.x = np.array([[x], # x
                           [y], # y
                           [0], # v_x
                           [0], # v_y
                           [0], # theta
                           [0]])# theta_dot

        self.F = np.array([[1, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0],
                           [0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 1]])

        self.B = np.array([[0, 0],
                           [0, 0],
                           [1, 1],
                           [1, 1],
                           [0, 0],
                           [1, -1]])

        self.u = np.array([[0, 0]]).transpose()
        self.b1 = 0.01
        self.b2 = 1
        self.time = time.time()
    def getInput(self):
        if pygame.key.get_pressed()[pygame.K_u]:
            self.u[0, 0] = 1
        elif pygame.key.get_pressed()[pygame.K_j]:
            self.u[0, 0] = -1
        else:
            self.u[0, 0] = 0

        if pygame.key.get_pressed()[pygame.K_i]:
            self.u[1, 0] = 1
        elif pygame.key.get_pressed()[pygame.K_k]:
            self.u[1, 0] = -1
        else:
            self.u[1, 0] = 0

    def predict(self):
        newTime = time.time()
        dt = newTime - self.time
        self.time = newTime

        theta = self.x[4,0]
        delta_th = 1-dt*self.b2
        vel = np.sqrt(self.x[2,0]**2 + self.x[3,0]**2)

        self.F = np.array([[1, 0, dt, 0, 0, 0],
                           [0, 1, 0, dt, 0, 0],
                           [0, 0, 1-dt*vel*self.b1, 0, 0, 0],
                           [0, 0, 0, 1-dt*vel*self.b1, 0, 0],
                           [0, 0, 0, 0, 1, dt],
                           [0, 0, 0, 0, 0, delta_th]])

        self.B = np.array([[0, 0],
                           [0, 0],
                           [300*dt*np.cos(theta), 300*dt*np.cos(theta)],
                           [300*dt*np.sin(theta), 300*dt*np.sin(theta)],
                           [0, 0],
                           [0.5*dt*self.w/2, -0.5*dt*self.w/2]])
        print(theta)

        self.x = self.F@self.x + self.B@self.u
    def draw(self):
        pygame.draw.circle(screen, "blue", (self.x[0,0], self.x[1,0]), self.w)
        pygame.draw.line(
            screen,
            "red",
            (self.x[0,0], self.x[1,0]),
            (self.x[0,0] + self.w*np.cos(self.x[4,0]), self.x[1,0] + self.w*np.sin(self.x[4,0]))
        )

if __name__ == "__main__":
    boat = Boat(300, 300, 20)
    while running:

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break

        screen.fill("white")
        # Render everything

        boat.getInput()
        boat.predict()
        boat.draw()

        # Code done

        pygame.display.flip()
    pygame.quit()
