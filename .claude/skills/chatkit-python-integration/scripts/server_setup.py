"""
Server setup script for ChatKit integration with FastAPI
"""
from fastapi import FastAPI, Request
from fastapi.responses import Response, StreamingResponse
from chatkit.server import ChatKitServer
from chatkit.stores import PostgresStore, BlobStorageStore
from chatkit.models import StreamingResult


def create_chatkit_server():
    """
    Creates and configures a ChatKit server with FastAPI
    """
    app = FastAPI()
    data_store = PostgresStore()
    attachment_store = BlobStorageStore(data_store)
    server = MyChatKitServer(data_store, attachment_store)

    @app.post("/chatkit")
    async def chatkit_endpoint(request: Request):
        result = await server.process(await request.body(), {})
        if isinstance(result, StreamingResult):
            return StreamingResponse(result, media_type="text/event-stream")
        else:
            return Response(content=result.json, media_type="application/json")

    return app


class MyChatKitServer(ChatKitServer):
    """
    Custom ChatKit server implementation
    """
    def __init__(self, data_store, attachment_store=None):
        super().__init__(data_store, attachment_store)

    # Additional implementation details would go here
    pass