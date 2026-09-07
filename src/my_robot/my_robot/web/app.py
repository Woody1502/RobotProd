"""FastAPI application factory."""

import asyncio
from pathlib import Path

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

from .node import RobotNode
from .routes import commands, gamepad, stream

# Static files are served by nginx in production.
# FastAPI mounts them as a fallback (e.g. direct access to port 8080).
_STATIC_DIR = Path(__file__).resolve().parent / 'static'


def create_app(node: RobotNode) -> FastAPI:
    app = FastAPI(docs_url=None, redoc_url=None)

    @app.on_event('startup')
    async def _startup():
        node._loop = asyncio.get_running_loop()

    app.include_router(commands.make_router(node))
    app.include_router(gamepad.make_router(node))
    app.include_router(stream.make_router(node))

    if _STATIC_DIR.exists():
        app.mount('/', StaticFiles(directory=str(_STATIC_DIR), html=True), name='static')

    return app
