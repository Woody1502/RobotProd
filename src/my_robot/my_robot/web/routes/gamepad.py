from fastapi import APIRouter
from ..node import RobotNode


def make_router(node: RobotNode) -> APIRouter:
    r = APIRouter()

    @r.post('/gamepad/enabled/{value}')
    async def gamepad_enabled(value: int):
        node.set_gamepad_enabled(bool(value))
        return {'ok': True}

    @r.post('/autopilot/enabled/{value}')
    async def autopilot_enabled(value: int):
        node.set_autopilot_enabled(bool(value))
        return {'ok': True}

    return r
