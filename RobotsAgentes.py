# ESTE PROGRAMA CONTIENE EL MODELO Y LOS AGENTES PARA SIMULAR LA INTERACCION DE ROBOTS
# DE CARGA QUE TRATAN DE APILAR CAJAS EN STACKS DE MAXIMO 5 UNIDADES

# Se importan todos los componentes necesarios
from random import randint
from mesa import Agent, Model
from mesa.space import MultiGrid
from mesa.time import RandomActivation
from mesa.visualization.modules import CanvasGrid, TextElement
from pathfinding.core.grid import Grid as Path_grid
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.finder.a_star import AStarFinder
import numpy
import math


# Se crean los modelos para los agentes involucrados
class Box(Agent):
    def __init__(self, model, pos):
        super().__init__(model.next_id(), model)
        self.pos = pos
        self.active = True


class Stack2(Agent):
    def __init__(self, model, pos):
        super().__init__(model.next_id(), model)
        self.pos = pos
        self.active = True


class Stack3(Agent):
    def __init__(self, model, pos):
        super().__init__(model.next_id(), model)
        self.pos = pos
        self.active = True


class Stack4(Agent):
    def __init__(self, model, pos):
        super().__init__(model.next_id(), model)
        self.pos = pos
        self.active = True


class Stack5(Agent):
    def __init__(self, model, pos):
        super().__init__(model.next_id(), model)
        self.pos = pos
        self.active = True


class Robot(Agent):
    def __init__(self, model, pos, maze, matrix, boxesList, stack2, stack3, stack4, stack5):
        super().__init__(model.next_id(), model)
        self.pos = pos
        self.lifting = False
        self.boxesList = boxesList
        self.stack2 = stack2
        self.stack3 = stack3
        self.stack4 = stack4
        self.stack5 = stack5
        self.maze = maze
        self.matrix = matrix
        self.numBoxes = len(boxesList)

    # Esta funcion recibe una lista de coordenadas y un agente tipo robot, para determinar la ubicacion
    # mas cercana y seguir la ruta marcada por el algoritmo A*
    def getShortestRoute(self, posList):
        shortestPath = 400
        for pos in posList:
            distance = math.sqrt(math.pow((pos[0] - self.pos[0]), 2) + math.pow((pos[1] - self.pos[1]), 2))
            if (distance < shortestPath):
                shortestPath = distance
                target = pos
        grid = Path_grid(matrix=self.maze)
        start = grid.node(self.pos[0], self.pos[1])
        finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
        endpoint = grid.node(target[0], target[1])
        route, runs = finder.find_path(start, endpoint, grid)
        return route, target

    def step(self):
        # En caso de no estar cargando una caja, el robot revisa la lista de cajas activas en la simulacion
        # y sigue la ruta marcada por el algoritmo A* para alcanzar la mas cercana
        if (self.lifting == False):
            shortestPath = 400
            target = self
            for box in self.boxesList:
                distance = math.sqrt(math.pow((box.pos[0] - self.pos[0]), 2) + math.pow((box.pos[1] - self.pos[1]), 2))
                if (distance < shortestPath):
                    shortestPath = distance
                    target = box
            grid = Path_grid(matrix=self.maze)
            start = grid.node(self.pos[0], self.pos[1])
            finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
            endpoint = grid.node(target.pos[0], target.pos[1])
            route, runs = finder.find_path(start, endpoint, grid)
            if (route[0] != target.pos):
                self.model.grid.move_agent(self, route[1])
                self.model.agentsMoves += 1
            else:
                agentsList = self.model.grid.get_cell_list_contents(self.pos)
                for agent in agentsList:
                    if (type(agent) == Box and agent.active):
                        agent.active = False
                        self.boxesList.remove(target)
                        self.lifting = True
                        self.model.agentsMoves += 1
                        break
        # Si el robot ya se encuentra cargando una caja, va en busca de stacks, priorizando los que tienen mayor
        # numero de cajas
        else:
            # Si la lista de stacks con 4 cajas no esta vacia, el robot utiliza la funcion 'getShortestRoute' con
            # dicha lista
            if (len(self.stack4) != 0):
                route, target = self.getShortestRoute(self.stack4)
                if (route[0] != target):
                    self.model.grid.move_agent(self, route[1])
                    self.model.agentsMoves += 1
                else:
                    agentsList = self.model.grid.get_cell_list_contents(self.pos)
                    for agent in agentsList:
                        if (type(agent) == Stack4 and agent.active):
                            # En caso de haberse encontrado el stack con 4 cajas, este se remueve de la lista y se
                            # añade un nuevo stack de 5 cajas, pues el robot en cuestion "suelta su caja"
                            agent.active = False
                            self.stack4.remove(target)
                            self.stack5.append(target)
                            self.lifting = False
                            self.model.agentsMoves += 1
                            break
            # Si la lista de stacks con 4 cajas esta vacia, se checa la lista de stacks con 3 cajas para revisar si
            # hay alguna. En caso afrimativo, el robot utiliza la funcion 'getShortestRoute' con dicha lista
            elif (len(self.stack3) != 0):
                route, target = self.getShortestRoute(self.stack3)
                if (route[0] != target):
                    self.model.grid.move_agent(self, route[1])
                    self.model.agentsMoves += 1
                else:
                    agentsList = self.model.grid.get_cell_list_contents(self.pos)
                    for agent in agentsList:
                        if (type(agent) == Stack3 and agent.active):
                            # En caso de haberse encontrado el stack con 3 cajas, este se remueve de la lista y se
                            # añade un nuevo stack de 4 cajas, pues el robot en cuestion "suelta su caja"
                            agent.active = False
                            self.stack3.remove(target)
                            self.stack4.append(target)
                            self.lifting = False
                            self.model.agentsMoves += 1
                            break
            # Si la lista de stacks con 3 cajas tambien esta vacia, se checa la lista de stacks con 2 cajas para revisar si
            # hay alguna. En caso afrimativo, el robot utiliza la funcion 'getShortestRoute' con dicha lista
            elif (len(self.stack2) != 0):
                route, target = self.getShortestRoute(self.stack2)
                if (route[0] != target):
                    self.model.grid.move_agent(self, route[1])
                    self.model.agentsMoves += 1
                else:
                    agentsList = self.model.grid.get_cell_list_contents(self.pos)
                    for agent in agentsList:
                        if (type(agent) == Stack2 and agent.active):
                            # En caso de haberse encontrado el stack con 2 cajas, este se remueve de la lista y se
                            # añade un nuevo stack de 3 cajas, pues el robot en cuestion "suelta su caja"
                            agent.active = False
                            self.stack2.remove(target)
                            self.stack3.append(target)
                            self.lifting = False
                            self.model.agentsMoves += 1
                            break
            # Si no se encuentra ningun tipo de stack, el robot vuelve a buscar en la lista de cajas activas en la
            # simulacion para hallar la mas cercana con 'getShortestRoute' y formar un stack de 2 cajas
            else:
                shortestPath = 400
                for box in self.boxesList:
                    distance = math.sqrt(
                        math.pow((box.pos[0] - self.pos[0]), 2) + math.pow((box.pos[1] - self.pos[1]), 2))
                    if (distance < shortestPath):
                        shortestPath = distance
                        target = box
                grid = Path_grid(matrix=self.maze)
                start = grid.node(self.pos[0], self.pos[1])
                finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
                endpoint = grid.node(target.pos[0], target.pos[1])
                route, runs = finder.find_path(start, endpoint, grid)
                if (route[0] != target.pos):
                    self.model.grid.move_agent(self, route[1])
                    self.model.agentsMoves += 1
                else:
                    agentsList = self.model.grid.get_cell_list_contents(self.pos)
                    for agent in agentsList:
                        if (type(agent) == Box and agent.active):
                            # En caso de haberse encontrado la caja destino, esta se marca como inactiva y se
                            # añade un nuevo stack de 2 cajas, pues el robot en cuestion "suelta su caja"
                            agent.active = False
                            self.boxesList.remove(target)
                            self.stack2.append(target.pos)
                            self.lifting = False
                            self.model.agentsMoves += 1
                            break


# La funcion TotalMoves despliega en pantalla el numero de movimientos hechos por todos los robots (cada robot)
# suma un movimiento a la variable 'agentsMoves' solo si se desplazan, toman o sueltan una caja
class TotalMoves(TextElement):
    def __init__(self):
        pass

    def render(self, model):
        return "Movimientos hechos por todos los robots: " + str(model.agentsMoves) + "\n"


# Los 'TextElement' adicionales solo indican la simbologia de los colores de las casillas en el grid generado por
# Mesa
class BrownMeaning(TextElement):
    def __init__(self):
        pass

    def render(self, model):
        return "CAFE: Una sola caja"


class PurpleMeaning(TextElement):
    def __init__(self):
        pass

    def render(self, model):
        return "MORADO: Pila de 2 cajas"


class BlueMeaning(TextElement):
    def __init__(self):
        pass

    def render(self, model):
        return "AZUL: Pila de 3 cajas"


class OrangeMeaning(TextElement):
    def __init__(self):
        pass

    def render(self, model):
        return "NARANJA: Pila de 4 cajas"


class GreenMeaning(TextElement):
    def __init__(self):
        pass

    def render(self, model):
        return "VERDE: Pila de 5 cajas"


# Se define el modelo para la simulacion llamado 'StockRoom'
class StockRoom(Model):
    def __init__(self, ratio=0.1, maxSteps=250):
        super().__init__()
        self.schedule = RandomActivation(self)
        self.grid = MultiGrid(20, 20, torus=False)
        self.maze = numpy.ones([20, 20], dtype=int)
        self.matrix = numpy.zeros((20, 20))
        self.maxSteps = maxSteps
        self.tilesWithBox = 400 * ratio
        self.boxesList = []
        self.stack2 = []
        self.stack3 = []
        self.stack4 = []
        self.stack5 = []
        self.agentsMoves = 0
        auxiliarTilesWithBox = self.tilesWithBox

        # Se toman casillas aleatorias para colocar todas las cajas necesarias (al principio solo puede
        # colocarse una caja por casilla)
        while (auxiliarTilesWithBox > 0):
            column = randint(0, 19)
            row = randint(0, 19)
            if (self.matrix[column][row] == 0):
                self.matrix[column][row] = 1
                auxiliarTilesWithBox -= 1

        # Con la referencia del grid, se instancian y posicionan todos los agentes tipo Box en las casillas
        # marcadas con el numero 1
        for _, x, y in self.grid.coord_iter():
            if self.matrix[y][x] == 1:
                newBox = Box(self, (y, x))
                self.boxesList.append(newBox)
                self.grid.place_agent(newBox, newBox.pos)
                self.schedule.add(newBox)

        # Los 5 agentes tipo robot se instancian y se colocan en casillas vacias diferentes
        for n in range(5):
            column = randint(0, 19)
            row = randint(0, 19)
            while (self.grid.get_cell_list_contents((column, row))):
                column = randint(0, 19)
                row = randint(0, 19)
            newRobot = Robot(self, (column, row), self.maze, self.matrix, self.boxesList, self.stack2, self.stack3,
                             self.stack4, self.stack5)
            self.grid.place_agent(newRobot, newRobot.pos)
            self.schedule.add(newRobot)

    # La funcion step permite el progreso de la simulacion y de todos sus agentes
    def step(self):
        # Si se alcanza el mayor numero de pasos permitido, la simulacion se detiene
        if (self.schedule.steps >= self.maxSteps):
            self.running = False
        # Si la simulacion sigue activa y todas las cajas ya se encuentran en stacks de 5, se instancian
        # dichos stacks por ultima vez y se detiene la simulacion
        elif (len(self.stack5) * 5 >= self.tilesWithBox):
            for stack in self.stack5:
                newStack5 = Stack5(self, stack)
                self.grid.place_agent(newStack5, stack)
                self.schedule.add(newStack5)
            self.schedule.step()
            self.running = False
        # Si la simulacion sigue activa pero todavia hay cajas que no estan en stacks de 5, se instancian
        # todos los stacks restantes (los de 2, 3 y 4 cajas)
        else:
            for stack in self.stack2:
                newStack2 = Stack2(self, stack)
                self.grid.place_agent(newStack2, stack)
                self.schedule.add(newStack2)
            for stack in self.stack3:
                newStack3 = Stack3(self, stack)
                self.grid.place_agent(newStack3, stack)
                self.schedule.add(newStack3)
            for stack in self.stack4:
                newStack4 = Stack4(self, stack)
                self.grid.place_agent(newStack4, stack)
                self.schedule.add(newStack4)
            for stack in self.stack5:
                newStack5 = Stack5(self, stack)
                self.grid.place_agent(newStack5, stack)
                self.schedule.add(newStack5)
            self.schedule.step()


""" #La funcion 'agent_portrayal' identifica los agentes existentes y los renderea conforme a su tipo
def agent_portrayal(agent):
    if(type(agent) == Robot):
        if(agent.lifting):
            return {"Shape": "RobotCargandoCaja.png", "Layer": 1}
        else:
            return {"Shape": "RobotSinCaja.png", "Layer": 1}
    if(type(agent) == Box and agent.active == True):
        return {"Shape": "rect", "w": 1, "h": 1, "Filled": "true", "Color": "Brown", "Layer": 0}
    if(type(agent) == Stack2 and agent.active == True):
        return {"Shape": "rect", "w": 1, "h": 1, "Filled": "true", "Color": "Purple", "Layer": 0}
    if(type(agent) == Stack3 and agent.active == True):
        return {"Shape": "rect", "w": 1, "h": 1, "Filled": "true", "Color": "Blue", "Layer": 0}
    if(type(agent) == Stack4 and agent.active == True):
        return {"Shape": "rect", "w": 1, "h": 1, "Filled": "true", "Color": "Orange", "Layer": 0}
    if(type(agent) == Stack5 and agent.active == True):
        return {"Shape": "rect", "w": 1, "h": 1, "Filled": "true", "Color": "Green", "Layer": 0}

#Se crean todos los elementos que seran desplegados en pantalla
grid = CanvasGrid(agent_portrayal, 20, 20, 500, 500)
brownText = BrownMeaning()
purpleText = PurpleMeaning()
blueText = BlueMeaning()
orangeText = OrangeMeaning()
greenText = GreenMeaning()
metricsText = TotalMoves()
#El server recibe todos los parametros necesarios
server = ModularServer(StockRoom, 
                        [grid, metricsText, brownText, purpleText, blueText, orangeText, greenText], 
                        "Robots de Carga", 
                        {"ratio": UserSettableParameter("slider", "Porcentaje de celdas con caja", 0.1, 0.05, 0.95, 0.05),
                        "maxSteps": UserSettableParameter("slider", "Maximo numero de pasos", 250, 1, 1000, 1)
        })
#Se selecciona el puerto
server.port = 8523
#Se lanza el servidor
server.launch() """