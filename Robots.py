from mesa import Model
from mesa.agent import Agent
from mesa.space import MultiGrid
from mesa.time import SimultaneousActivation
from mesa.datacollection import DataCollector
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from mesa.time import RandomActivation
from random import randint

import numpy as np

class Shelf(Agent):
    def __init__(self, unique_id, model, pos):
        super().__init__(unique_id, model)
        self.pos = pos
        self.is_occupied = False
        self.stored_box = None

    def step(self):
        # Lógica para actualizar el estado del estante
        if self.stored_box is not None:
            self.is_occupied = True
        else:
            self.is_occupied = False

    def store_box(self, box):
        if not self.is_occupied:
            self.stored_box = box
            self.is_occupied = True
            return True
        return False

    def remove_box(self):
        if self.is_occupied:
            box = self.stored_box
            self.stored_box = None
            self.is_occupied = False
            return box
        return None


class Box(Agent):
    def __init__(self, unique_id, model, pos, destination=None):
        super().__init__(unique_id, model)
        self.pos = pos
        self.destination = destination

    def set_destination(self, destination):
        self.destination = destination

    def get_destination(self):
        return self.destination


class Robot(Agent):
    def __init__(self, unique_id, model, pos):
        super().__init__(unique_id, model)
        self.pos = pos
        self.carrying_box = None

    def step(self):
        if self.carrying_box is None:
            box = self.find_nearest_box()
            if box is not None:
                self.pick_up_box(box)
        else:
            self.deliver_box_to_shelf()

    def find_nearest_box(self):
        nearest_box = None
        min_distance = float('inf')

        for box in self.model.boxes:  # Asegúrate de que model.boxes esté sincronizado con Unity
            if not box.is_in_shelf:  # Verifica el estado de la caja
                distance = self.calculate_distance(self.pos, box.pos)
                if distance < min_distance:
                    min_distance = distance
                    nearest_box = box

        return nearest_box

    @staticmethod
    def calculate_distance(pos1, pos2):
        # Calcula la distancia Euclidiana (o Manhattan si es más apropiada para tu simulación)
        return ((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2) ** 0.5

    def pick_up_box(self, box):
        # Implementar lógica para "recoger" la caja (simulada por una animación en Unity)
        self.carrying_box = box
        # Actualizar la posición de la caja (opcional, dependiendo de cómo manejes las animaciones)
        box.pos = self.pos

    def deliver_box_to_shelf(self):
        # Paso 1: Encontrar el estante más cercano disponible
        shelf = self.find_nearest_available_shelf()
        if shelf is None:
            return  # No hay estantes disponibles

        # Paso 2: Calcular la ruta más rápida hacia el estante
        route = self.find_fastest_route(shelf.pos)
        if route is None:
            return  # No se encontró ruta

        # Paso 3: Mover el robot a lo largo de la ruta
        self.move_along_route(route)

        # Paso 4: Entregar la caja al estante
        if self.pos == shelf.pos:
            self.place_box_on_shelf()
            self.carrying_box = None  # El robot ya no lleva la caja

    def find_nearest_available_shelf(self):
        nearest_shelf = None
        min_distance = float('inf')

        for shelf in self.model.shelvesList:  # Asumiendo que model.shelvesList es la lista de todos los estantes
            if not shelf.is_occupied:  # Suponiendo que los estantes tienen un atributo para saber si están ocupados
                distance = self.calculate_distance(self.pos, shelf.pos)
                if distance < min_distance:
                    min_distance = distance
                    nearest_shelf = shelf

        return nearest_shelf

    def find_fastest_route(self, destination):
        # Crear una cuadrícula basada en el entorno actual
        # Aquí, 'matrix' es una representación de la cuadrícula de tu entorno con 0 para espacios libres y 1 para obstáculos
        grid = Grid(matrix=self.model.matrix)

        start = grid.node(self.pos[0], self.pos[1])
        end = grid.node(destination[0], destination[1])

        # Inicializar el buscador A*
        finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
        path, _ = finder.find_path(start, end, grid)

        return path

    def move_along_route(self, route):
        if not route:  # Si la ruta está vacía, no hay nada que hacer
            return

        # Asumiendo que la ruta es una lista de tuplas (x, y)
        # El primer elemento de la ruta es la posición actual, por lo que empezamos con el segundo
        for next_step in route[1:]:
            # Mover el robot a la siguiente posición
            self.move_to(next_step)

            # Puedes añadir una verificación para detener el movimiento si se alcanza el destino o si hay algún obstáculo
            if self.pos == next_step:
                break

    def move_to(self, next_step):
        # Actualiza la posición del robot en la cuadrícula
        # Aquí debes manejar la lógica para actualizar la posición del robot, incluyendo la verificación de obstáculos y otros agentes
        self.model.grid.move_agent(self, next_step)
        self.pos = next_step

    def place_box_on_shelf(self):
        # Asegúrate de que el robot esté llevando una caja
        if self.carrying_box is None:
            return

        # Encuentra el estante en la posición actual del robot
        shelf = self.get_shelf_at_position(self.pos)
        if shelf is not None and not shelf.is_occupied:
            # Coloca la caja en el estante y actualiza el estado de ambos
            shelf.store_box(self.carrying_box)
            self.carrying_box = None  # El robot ya no está llevando la caja

    def get_shelf_at_position(self, pos):
        # Busca un estante en la posición dada
        contents = self.model.grid.get_cell_list_contents(pos)
        shelves = [obj for obj in contents if isinstance(obj, Shelf)]
        return shelves[0] if shelves else None

#ALMACEN
class WarehouseModel(Model):
    def __init__(self, ratio=0.1, maxSteps=250):
        super().__init__()
        self.schedule = RandomActivation(self)
        self.grid = MultiGrid(20, 20, torus=False)
        self.maxSteps = maxSteps
        self.numBoxes = int(400 * ratio)
        self.boxesList = []
        self.shelvesList = []

        # Colocar cajas en ubicaciones aleatorias
        for _ in range(self.numBoxes):
            x, y = randint(0, 19), randint(0, 19)
            while self.grid.is_cell_empty((x, y)) == False:
                x, y = randint(0, 19), randint(0, 19)
            newBox = Box(self, (x, y))
            self.boxesList.append(newBox)
            self.grid.place_agent(newBox, (x, y))
            self.schedule.add(newBox)

        # Colocar robots en ubicaciones aleatorias
        for i in range(5):
            x, y = randint(0, 19), randint(0, 19)
            while self.grid.is_cell_empty((x, y)) == False:
                x, y = randint(0, 19), randint(0, 19)
            newRobot = Robot(self, (x, y), self.boxesList, self.shelvesList)
            self.grid.place_agent(newRobot, (x, y))
            self.schedule.add(newRobot)

        # Definir ubicaciones de estantes de forma estática
        shelf_positions = [(1, 1), (5, 5), (10, 10), (15, 15), (19, 19)]  # Ejemplo de ubicaciones

        # Crear y colocar estantes en esas ubicaciones
        for pos in shelf_positions:
            newShelf = Shelf('ID', self, pos)  # 'ID' debe ser un identificador único
            self.shelvesList.append(newShelf)
            self.grid.place_agent(newShelf, pos)
            self.schedule.add(newShelf)

    def step(self):
        if self.schedule.steps >= self.maxSteps:
            self.running = False
        # Aquí puedes agregar cualquier lógica adicional específica para cada paso
        else:
            self.schedule.step()

