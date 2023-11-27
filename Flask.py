import flask
from flask.json import jsonify
import uuid
from Robots import WarehouseModel, Robot, Box, Shelf  # Asegúrate de importar las clases correctas

robotsSimulations = {}

app = flask.Flask(__name__)


@app.route("/warehouseSimulations", methods=["POST"])
def create():
    global robotsSimulations
    id = str(uuid.uuid4())
    robotsSimulations[id] = WarehouseModel()  # Asume que WarehouseModel es tu modelo de simulación
    return "ok", 201, {'Location': f"/warehouseSimulations/{id}"}


@app.route("/warehouseSimulations/<id>", methods=["GET"])
def queryState(id):
    global model
    model = robotsSimulations[id]
    model.step()
    content = {"robots": [], "boxes": [], "shelves": []}

    for agent in model.schedule.agents:
        if isinstance(agent, Robot):
            content["robots"].append(
                {"id": agent.unique_id, "x": agent.pos[0], "y": agent.pos[1]})
        elif isinstance(agent, Box):
            content["boxes"].append(
                {"id": agent.unique_id, "x": agent.pos[0], "y": agent.pos[1]})
        elif isinstance(agent, Shelf):
            content["shelves"].append(
                {"id": agent.unique_id, "x": agent.pos[0], "y": agent.pos[1]})

    return jsonify(content)


app.run()
