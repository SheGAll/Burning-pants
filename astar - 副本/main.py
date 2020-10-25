from vision import Vision
from action import Action
from debug import Debugger
from astar import AStarPlanner
import math
import time

import math


def turnto(result_orientation):
    # print('ro=' + str(result_orientation))
    while abs(result_orientation - vision.blue_robot[0].orientation) > 0.05:
        # print('turned')
        if (math.pi > result_orientation > math.pi / 2) and (
                -math.pi < vision.blue_robot[0].orientation < -math.pi / 2):
            action.sendCommand(vx=60, vy=60, vw=-math.pi / 4)
            time.sleep(math.pi / 500)
            continue
        if (math.pi > vision.blue_robot[0].orientation > math.pi / 2) and (
                -math.pi < result_orientation < -math.pi / 2):
            action.sendCommand(vx=60, vy=60, vw=math.pi / 4)
            time.sleep(math.pi / 500)
            continue
        if result_orientation - vision.blue_robot[0].orientation <= 0:
            action.sendCommand(vx=60, vy=60, vw=-math.pi)
            time.sleep(math.pi / 500)
            continue
        else:
            action.sendCommand(vx=60, vy=60, vw=math.pi)
            time.sleep(math.pi / 500)
    # print('vo=' + str(vision.blue_robot[0].orientation))


def judge_next_node(id, nlist):
    rx = vision.blue_robot[0].x
    ry = vision.blue_robot[0].y
    rt = 0.1
    if rx > nlist[id][0] and ry > nlist[id][1]:
        turnto(-math.pi + math.atan(abs(ry - nlist[id][1]) / abs(rx - nlist[id][0])))
        straight(rt)
        return
    if rx == nlist[id][0] and ry > nlist[id][1]:
        turnto(-math.pi / 2)
        straight(rt)
        return
    if rx < nlist[id][0] and ry > nlist[id][1]:
        turnto(-math.atan(abs(ry - nlist[id][1]) / abs(rx - nlist[id][0])))
        straight(rt)
        return
    if rx < nlist[id][0] and ry == nlist[id][1]:
        turnto(0)
        straight(rt)
        return
    if rx < nlist[id][0] and ry < nlist[id][1]:
        turnto(math.atan(abs(ry - nlist[id][1]) / abs(rx - nlist[id][0])))
        straight(rt)
        return
    if rx == nlist[id][0] and ry < nlist[id][1]:
        turnto(math.pi / 2)
        straight(rt)
        return
    if rx > nlist[id][0] and ry < nlist[id][1]:
        turnto(math.pi - math.atan(abs(ry - nlist[id][1]) / abs(rx - nlist[id][0])))
        straight(rt)
        return
    if rx > nlist[id][0] and ry == nlist[id][1]:
        turnto(math.pi)
        straight(rt)
        return


def straight(rt):
    action.sendCommand(vx=60,
                       vy=60, vw=0)
    time.sleep(rt)
    return


def goto(id, nlist):
    while math.hypot(nlist[id][0] - vision.blue_robot[0].x, nlist[id][1] - vision.blue_robot[0].y) > 10:
        judge_next_node(id, nlist)


def run_next_node(id, nlist, num):
    # print(math.hypot(-2400 - vision.blue_robot[0].x, -1500 - vision.blue_robot[0].y))
    if math.hypot(-2400 - vision.blue_robot[0].x, -1500 - vision.blue_robot[0].y) < 50 and num == 0:
        action.sendCommand(vx=0, vy=0, vw=0)
        return
    if math.hypot(2400 - vision.blue_robot[0].x, 1500 - vision.blue_robot[0].y) < 50 and num == 1:
        action.sendCommand(vx=0, vy=0, vw=0)
        return
    if id + 1 <= len(nlist):
        goto(id, nlist)
        # print(id)
        run_next_node(id + 1, nlist, num)
    else:
        return


if __name__ == '__main__':
    vision = Vision()
    action = Action()
    debugger = Debugger()
    planner = AStarPlanner()
    n = 0
    while True:
        for i in range(20):
            # Path planning
            planner = AStarPlanner(ox=planner.get_obstacle_point(vision)[0], oy=planner.get_obstacle_point(vision)[1])
            start_x, start_y = vision.my_robot.x, vision.my_robot.y
            goal_x, goal_y = -2400, -1500
            path_x, path_y = planner.planning(sx=start_x, sy=start_y, gx=goal_x, gy=goal_y)

            debugger.draw_all(path_x, path_y)
            path_node = list(zip(path_x, path_y))

        run_next_node(0, path_node[::-1], 0)
        run_next_node(0, path_node, 1)
        n = n + 1
        print(n)
        time.sleep(5)
