import asyncio
import json

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from fastapi.responses import StreamingResponse
from ..node import RobotNode


def make_router(node: RobotNode) -> APIRouter:
    r = APIRouter()

    def _mjpeg_gen(lock, getter):
        async def _gen():
            while True:
                with lock:
                    jpeg = getter()
                if jpeg:
                    yield (
                        b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n\r\n'
                        + jpeg +
                        b'\r\n'
                    )
                await asyncio.sleep(0.033)
        return _gen()

    @r.get('/video')
    async def video():
        return StreamingResponse(
            _mjpeg_gen(node._frame_lock, lambda: node._latest_jpeg),
            media_type='multipart/x-mixed-replace; boundary=frame',
        )

    @r.get('/video/mask')
    async def video_mask():
        return StreamingResponse(
            _mjpeg_gen(node._graphic_lock, lambda: node._latest_graphic),
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
