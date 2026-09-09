import asyncio
import json

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from fastapi.responses import Response
from ..node import RobotNode

_NO_CACHE = {'Cache-Control': 'no-store, no-cache', 'Pragma': 'no-cache'}


def make_router(node: RobotNode) -> APIRouter:
    r = APIRouter()

    @r.get('/snapshot')
    async def snapshot():
        with node._frame_lock:
            data = node._latest_jpeg
        if not data:
            return Response(status_code=204)
        return Response(content=data, media_type='image/jpeg', headers=_NO_CACHE)

    @r.get('/snapshot/mask')
    async def snapshot_mask():
        with node._graphic_lock:
            data = node._latest_graphic
        if not data:
            return Response(status_code=204)
        return Response(content=data, media_type='image/jpeg', headers=_NO_CACHE)

    async def _video_stream(websocket: WebSocket, lock, attr):
        await websocket.accept()
        try:
            while True:
                with lock:
                    data = getattr(node, attr)
                if data:
                    await websocket.send_bytes(data)
                await asyncio.sleep(0.033)
        except (WebSocketDisconnect, Exception):
            pass

    @r.websocket('/ws/camera')
    async def ws_camera(websocket: WebSocket):
        await _video_stream(websocket, node._frame_lock, '_latest_jpeg')

    @r.websocket('/ws/camera/mask')
    async def ws_camera_mask(websocket: WebSocket):
        await _video_stream(websocket, node._graphic_lock, '_latest_graphic')

    @r.websocket('/ws')
    async def ws(websocket: WebSocket):
        await websocket.accept()
        node._ws_clients.add(websocket)
        await websocket.send_text(json.dumps(node._status))
        try:
            while True:
                await websocket.receive_text()
        except (WebSocketDisconnect, Exception):
            pass
        finally:
            node._ws_clients.discard(websocket)

    return r
