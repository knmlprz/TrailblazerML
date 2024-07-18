from fastapi import FastAPI
import json
from pydantic import BaseModel
import uvicorn
from pathlib import Path
app = FastAPI()

class Coordinates(BaseModel):
    latitude: float
    longitude: float

@app.get("/v1/position")
async def get_position():
    try:
        with open("utils/position.json", "r") as file:
            position = json.load(file)
        return position
    except FileNotFoundError:
        return {"message": "Position file not found."}

@app.post("/v1/target")
async def set_target(target: Coordinates):
    # Użycie absolutnej ścieżki do pliku
    with open("utils/target.json", "w") as file:
        json.dump(target.dict(), file)
    return {"message": "Target saved successfully."}

def run_api():
    uvicorn.run(app, host="0.0.0.0", port=8899)
