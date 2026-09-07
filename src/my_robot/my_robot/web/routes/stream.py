import asyncio
import json

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from fastapi.responses import StreamingResponse
from ..node import RobotNode


def make_router(node: RobotNode) -> APIRouter:
    r = APIRouter()

    @r.get('/video')
    async def video():
        async def _mjpeg():
            while True:
                with node._frame_lock:
                    jpeg = node._latest_jpeg
                if jpeg:
                    yield (
                        b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n\r\n'
                        + jpeg +
                        b'\r\n'
                    )
                await asyncio.sleep(0.033)

        return StreamingResponse(
            _mjpeg(),
            media_type='multipart/x-mixed-replace; boundary=frame',
        )

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
