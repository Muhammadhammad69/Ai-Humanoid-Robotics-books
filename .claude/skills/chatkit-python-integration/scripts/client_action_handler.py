"""
Client-side action handling for ChatKit integration
"""
from collections.abc import AsyncIterator
from datetime import datetime
from typing import Any, Dict, Optional
from chatkit.actions import Action
from chatkit.server import ChatKitServer
from chatkit.types import (
    HiddenContextItem,
    ThreadItemDoneEvent,
    ThreadMetadata,
    ThreadStreamEvent,
    WidgetItem,
)


class MyChatKitServer(ChatKitServer):
    """
    ChatKit server with client action handling capabilities
    """
    async def action(
        self,
        thread: ThreadMetadata,
        action: Action[str, Any],
        sender: WidgetItem | None,
        context: Dict[str, Any],
    ) -> AsyncIterator[ThreadStreamEvent]:
        """
        Handle incoming actions from client-side
        """
        if action.type == "example":
            await self.do_thing(action.payload.get('id'))

            # Add hidden context item so the model can see the user action
            hidden = HiddenContextItem(
                id=self.store.generate_item_id("message", thread, context),
                thread_id=thread.id,
                created_at=datetime.now(),
                content=[f"<USER_ACTION>The user did a thing with id {action.payload.get('id')}</USER_ACTION>"]
            )
            await self.store.add_thread_item(thread.id, hidden, context)

            # Run inference to stream response back to user
            async for event in self.generate_response(context, thread):
                yield event

        elif action.type == "update_todo":
            # Handle todo update action
            item_id = action.payload.get('id')
            title = action.payload.get('title')
            description = action.payload.get('description')

            # Process the todo update
            await self.update_todo(item_id, title, description)

            # Add confirmation to thread
            hidden = HiddenContextItem(
                id=self.store.generate_item_id("message", thread, context),
                thread_id=thread.id,
                created_at=datetime.now(),
                content=[f"<TODO_UPDATED>ID: {item_id}, Title: {title}</TODO_UPDATED>"]
            )
            await self.store.add_thread_item(thread.id, hidden, context)

            async for event in self.generate_response(context, thread):
                yield event

        # Handle other action types as needed
        # ...

    async def do_thing(self, item_id: Optional[str]):
        """
        Example action processing function
        """
        # Implementation for the action
        pass

    async def update_todo(self, item_id: str, title: str, description: str):
        """
        Update a todo item
        """
        # Implementation for updating a todo
        pass

    async def generate_response(self, context: Dict[str, Any], thread: ThreadMetadata):
        """
        Generate response for the thread
        """
        # Placeholder implementation - would connect to LLM
        pass