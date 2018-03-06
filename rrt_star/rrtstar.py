#!/usr/bin/env python

# brrtstar.py
# RRT*
# by Yan Wang
# March, 2018

import sys, random, math, pygame
from pygame.locals import *
from math import sqrt,cos,sin,atan2
from lineIntersect import *
import time

#constants
XDIM = 640
YDIM = 480
WINSIZE = [XDIM, YDIM]
EPSILON = 7
NUMNODES = 20000
RADIUS=30
OBS=[(300,350,250,50),(80,80,150,50),(150,220,250,50)]


def obsDraw(pygame,screen):
    blue=(0,0,255)
    for o in OBS:
        pygame.draw.rect(screen,blue,o)

def drawSolutionPath(p, pygame, screen):
    pink = 200, 20, 240
    if p != []:
        for i in range(len(p)-1):
            pygame.draw.line(screen, pink, [p[i].x, p[i].y], [p[i+1].x, p[i+1].y], 5)

def updateSolutionPath(p, pygame, screen, edges):
    pink = 200, 20, 240
    white = 255, 255, 255
    black = 20, 20, 40
    if p != []:
        screen.fill(white)
        obsDraw(pygame, screen)
        for e in edges:
            p1, p2 = e
            pygame.draw.line(screen, black, [p1.x, p1.y], [p2.x, p2.y])

        for i in range(len(p)-1):
            pygame.draw.line(screen, pink, [p[i].x, p[i].y], [p[i+1].x, p[i+1].y], 5)


class Node:
    x = 0
    y = 0
    cost=0
    parent=None
    def __init__(self,xcoord, ycoord):
         self.x = xcoord
         self.y = ycoord


class BrrtStar:
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal
        self.nodes = []
        self.T = []
        self.edges = []
        pygame.init()
        self.screen = pygame.display.set_mode(WINSIZE)
        pygame.display.set_caption('RRTstar')
        self.white = 255, 255, 255
        self.black = 20, 20, 40
        self.screen.fill(self.white)
        obsDraw(pygame, self.screen)

        self.start_time = time.time()

    def dist(self, n1, n2):
        return sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)

    def extend(self, n1, n2):
        if self.dist(n1, n2) < EPSILON:
            return n2
        else:
            theta = atan2(n2.y - n1.y, n2.x - n1.x)
            return Node(n1.x + EPSILON * cos(theta), n1.y + EPSILON * sin(theta))

    def steer(self, n1, n2):
            return (n1, n2)

    def obstacleFree(self, sigma):
        p1, p2 = sigma
        return checkIntersectPoints(p1.x, p1.y, p2.x, p2.y, OBS)

    def getSortedList1(self, x_rand, X_near, T):
        Ls = []
        for x in X_near:
            sigma = self.steer(x, x_rand)
            C = x.cost + self.dist(x, x_rand)
            Ls.append((x, C, sigma))
        sorted(Ls, key=lambda a: a[1])
        return Ls

    def getSortedList2(self, x_rand, X_near, T):
        Ls = []
        for x in X_near:
            sigma = self.steer(x, x_rand)
            C = x.cost + self.dist(x, x_rand) + x_rand.cost
            Ls.append((x, C, sigma))
        sorted(Ls, key=lambda a: a[1])
        return Ls

    def chooseBestParent(self, Ls):
        for x, C, sigma in Ls:
            if self.obstacleFree(sigma):
                return x
        return None

    def rewireVertices(self, x_rand, Ls):
        for x, C, sigma in Ls:
            if x_rand.cost + self.dist(x_rand, x) < x.cost:
                if self.obstacleFree(sigma):
                    pygame.draw.line(self.screen, self.white, [x.x, x.y], [x.parent.x, x.parent.y])
                    self.edges.remove((x.parent, x))
                    pygame.draw.line(self.screen, self.black, [x.x, x.y], [x_rand.x, x_rand.y])
                    x.parent = x_rand
                    self.edges.append((x.parent, x))

    def nearVertices(self, x_new, T):
        X_near = []
        for x in T:
            if self.dist(x_new, x) < RADIUS:
                X_near.append(x)
        return X_near

    def nearestVertex(self, x_new, T):
        x_nearest = T[0]
        min_dist = self.dist(x_nearest, x_new)
        for x in T:
            dist = self.dist(x, x_new)
            if dist < min_dist:
                min_dist = dist
                x_nearest = x
        return x_nearest

    def sample(self):
        return Node(random.random()*XDIM, random.random()*YDIM)

    def insertVertex(self, x_new, x_min, T):
        x_new.parent = x_min
        x_new.cost = x_min.cost + self.dist(x_new, x_min)
        T.append(x_new)
        self.nodes.append(x_new)
        self.edges.append((x_new.parent, x_new))
        pygame.draw.line(self.screen, self.black, [x_min.x, x_min.y], [x_new.x, x_new.y])

    def generatePath(self, x1, x2):
        p1 = []
        p2 = []
        n1 = x1
        n2 = x2
        p1.append(n1)
        while n1.parent != None:
            p1.append(n1.parent)
            n1 = n1.parent
        p1.reverse()

        p2.append(n2)
        while n2.parent != None:
            p2.append(n2.parent)
            n2 = n2.parent
        return p1 + p2

    def connect(self, x1, x2, T):
        x_new = self.extend(x2, x1)
        X_near = self.nearVertices(x_new, T)
        Ls = self.getSortedList2(x1, X_near, T)
        x_min = self.chooseBestParent(Ls)
        if x_min != None:
            self.edges.append((x_min, x1))
            #x_new.parent = x_min
            #x_new.cost = x_min.cost + self.dist(x_new, x_min)
            #T.append(x_new)
            sigma_f = self.generatePath(x_min, x1)
            return sigma_f
        return None

    def cost(self, p):
        if p == []:
            return 10000000000
        if self.start not in p or self.goal not in p:
            return 10000000000
        c = 0
        for i in range(len(p)-1):
            c += self.dist(p[i], p[i+1])
        return c

    def run(self):
        self.nodes = []
        self.nodes.append(self.start)
        self.nodes.append(self.goal)
        self.edges = []
        self.Ta = [self.start]
        self.Tb = [self.goal]
        self.sigma_best = []
        self.path_num = 0
        for i in range(NUMNODES):
            x_rand = self.sample()
            x_nearest = self.nearestVertex(x_rand, self.Ta)
            x_new = self.extend(x_nearest, x_rand)
            X_near = self.nearVertices(x_new, self.Ta)
            Ls = self.getSortedList1(x_new, X_near, self.Ta)
            x_min = self.chooseBestParent(Ls)
            if x_min != None:
                self.insertVertex(x_new, x_min, self.Ta)
                self.rewireVertices(x_new, Ls)
            x_conn = self.nearestVertex(x_new, self.Tb)
            sigma_new = self.connect(x_new, x_conn, self.Tb)
            if sigma_new != None and self.cost(sigma_new) < self.cost(self.sigma_best):
                self.sigma_best = sigma_new
                self.path_num += 1
                print 'path: {}, time: {}s, len: {}, node num: {}'.format(self.path_num, time.time() - self.start_time, self.cost(sigma_new), len(self.nodes))
                updateSolutionPath(self.sigma_best, pygame, self.screen, self.edges)
                pygame.image.save(self.screen, 'B-RRT*{}.png'.format(self.path_num))
            # swap trees
            T = self.Ta
            self.Ta = self.Tb
            self.Tb = T

            pygame.display.update()


def main():
    start=Node(0.0,0.0)
    goal=Node(630.0,470.0)
    brrt_star = BrrtStar(start, goal)
    brrt_star.run()


# if python says run, then we should run
if __name__ == '__main__':
    main()