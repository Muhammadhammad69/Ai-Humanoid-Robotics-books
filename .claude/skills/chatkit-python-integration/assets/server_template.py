"""
Complete ChatKit Server Template
This template provides a full implementation of a ChatKit server with FastAPI
"""

from fastapi import FastAPI, Request
from fastapi.responses import Response, StreamingResponse
from chatkit.server import ChatKitServer, ThreadStreamEvent, StreamingResult
from chatkit.models import UserMessageItem, ThreadMetadata
from chatkit.stores import Store, AttachmentStore, PostgresStore, BlobStorageStore
from agents.sdk.agent import Agent, AgentContext
from agents.sdk.runner import Runner
from agents.sdk.utils import simple_to_agent_input
from agents.sdk.streaming import stream_agent_response
from typing import Any, AsyncIterator
import asyncio


class MyChatKitServer(ChatKitServer):
    """
    Complete implementation of a ChatKit server with all necessary methods
    """
    def __init__(self, data_store: Store, attachment_store: AttachmentStore | None = None):
        super().__init__(data_store, attachment_store)

        # Initialize your agent here
        self.assistant_agent = Agent[AgentContext](
            model="gpt-4o",  # Use the appropriate model
            name="Assistant",
            instructions="You are a helpful assistant"
        )

    async def respond(
        self,
        thread: ThreadMetadata,
        input: UserMessageItem | None,
        context: Any,
    ) -> AsyncIterator[ThreadStreamEvent]:
        """
        Main method to respond to user messages
        """
        agent_context = AgentContext(
            thread=thread,
            store=self.store,
            request_context=context,
        )

        result = Runner.run_streamed(
            self.assistant_agent,
            await simple_to_agent_input(input) if input else [],
            context=agent_context,
        )

        async for event in stream_agent_response(agent_context, result):
            yield event

    async def action(
        self,
        thread: ThreadMetadata,
        action: Action[str, Any],
        sender: WidgetItem | None,
        context: RequestContext,
    ) -> AsyncIterator[ThreadStreamEvent]:
        """
        Handle custom actions from widgets
        """
        # Implement action handling logic here
        if action.type == "example":
            # Process the action
            await self.do_something(action.payload.get('id'))

            # Add hidden context so the model knows about the action
            hidden = HiddenContextItem(
                id=self.store.generate_item_id("message", thread, context),
                thread_id=thread.id,
                created_at=datetime.now(),
                content=[f"<USER_ACTION>Performed example action</USER_ACTION>"]
            )
            await self.store.add_thread_item(thread.id, hidden, context)

            # Stream a response back
            async for event in self.respond(thread, None, context):
                yield event

    async def do_something(self, item_id: str):
        """
        Example action implementation
        """
        # Your action logic here
        pass


# Initialize the server components
app = FastAPI()
data_store = PostgresStore()  # Replace with your actual store implementation
attachment_store = BlobStorageStore(data_store)  # Replace with your actual attachment store
server = MyChatKitServer(data_store, attachment_store)


@app.post("/chatkit")
async def chatkit_endpoint(request: Request):
    """
    Main endpoint for ChatKit integration
    """
    result = await server.process(await request.body(), {})
    if isinstance(result, StreamingResult):
        return StreamingResponse(result, media_type="text/event-stream")
    else:
        return Response(content=result.json, media_type="application/json")


# Additional endpoints can be added here as needed
@app.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {"status": "healthy"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)