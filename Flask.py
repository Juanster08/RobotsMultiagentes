# ESTE PROGRAMA ES EL BACKEND PARA PODER HACER INSTANCIAS DEL MODELO DE ROBOTS
# CARGADORES E INTERACTUAR CON LA SIMULACION A TRAVES DE REQUESTS

# Se importan todos los componentes necesarios
import flask
from flask.json import jsonify
import uuid
from RobotsAgentes import StockRoom, Robot, Box, Stack2, Stack3, Stack4, Stack5

robotsSimulations = {}

app = flask.Flask(__name__)


# Se indica la ruta para poder hacer una accion POST, que crea una nueva simulacion con un ID
# unico, se retorna un 201 indicando la correcta ejecucion, asi como la referencia al enlace para
# poder interactuar dicha simulacion
@app.route("/robotsSimulations", methods=["POST"])
def create():
    global robotsSimulations
    id = str(uuid.uuid4())
    robotsSimulations[id] = StockRoom()
    return "ok", 201, {'Location': f"/robotsSimulations/{id}"}


# Se indica la ruta para poder hacer una accion GET, que ejecuta un paso en el modelo de la simulacion y
# ademas retorna la informacion necesaria de todos los agentes en un JSON
@app.route("/robotsSimulations/<id>", methods=["GET"])
def queryState(id):
    global model
    model = robotsSimulations[id]
    model.step()
    content = {"robots": [], "boxes": [], "stacks2": [], "stacks3": [], "stacks4": [], "stacks5": []}

    for agent in model.schedule.agents:
        if isinstance(agent, Robot):
            content["robots"].append(
                {"id": agent.unique_id, "lifting": agent.lifting, "x": agent.pos[0], "z": agent.pos[1]})
        elif isinstance(agent, Box):
            content["boxes"].append(
                {"id": agent.unique_id, "active": agent.active, "x": agent.pos[0], "z": agent.pos[1]})
        elif isinstance(agent, Stack2):
            content["stacks2"].append(
                {"id": agent.unique_id, "active": agent.active, "x": agent.pos[0], "z": agent.pos[1]})
        elif isinstance(agent, Stack3):
            content["stacks3"].append(
                {"id": agent.unique_id, "active": agent.active, "x": agent.pos[0], "z": agent.pos[1]})
        elif isinstance(agent, Stack4):
            content["stacks4"].append(
                {"id": agent.unique_id, "active": agent.active, "x": agent.pos[0], "z": agent.pos[1]})
        else:
            content["stacks5"].append(
                {"id": agent.unique_id, "active": agent.active, "x": agent.pos[0], "z": agent.pos[1]})

    # Toda la informacion del diccionario 'content' es retornado con un format JSON
    return jsonify(content)


# La aplicacion es lanzada
app.run()