from fastapi import APIRouter
from ..node import RobotNode


def make_router(node: RobotNode) -> APIRouter:
    r = APIRouter()

    @r.post('/cmd/estop')
    async def estop():
        node.estop()
        return {'ok': True}

    @r.post('/cmd/{device}/{state}')
    async def cmd(device: str, state: int):
        node.cmd(device, state)
        return {'ok': True}

    return r
