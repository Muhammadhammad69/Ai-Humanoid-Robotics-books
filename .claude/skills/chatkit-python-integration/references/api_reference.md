# ChatKit Python API Reference

## Installation

```bash
pip install openai-chatkit
```

## Core Components

### ChatKitServer
The base server class that handles chat operations:

```python
from chatkit.server import ChatKitServer

class MyChatKitServer(ChatKitServer):
    def __init__(self, data_store, attachment_store=None):
        super().__init__(data_store, attachment_store)
```

### Store Interface
The abstract base class for data persistence:

```python
from abc import ABC
from typing import Generic, Literal, TypeVar

TContext = TypeVar("TContext")

class Store(ABC, Generic[TContext]):
    def generate_thread_id(self, context: TContext) -> str: ...
    def generate_item_id(self, item_type: Literal["message", "tool_call", "task", "workflow", "attachment"], thread: ThreadMetadata, context: TContext) -> str: ...
    async def load_thread(self, thread_id: str, context: TContext) -> ThreadMetadata: ...
    async def save_thread(self, thread: ThreadMetadata, context: TContext) -> None: ...
    # ... other methods
```

### AttachmentStore Interface
For handling file attachments:

```python
class AttachmentStore(ABC, Generic[TContext]):
    async def delete_attachment(self, attachment_id: str, context: TContext) -> None: ...
    async def create_attachment(self, input: AttachmentCreateParams, context: TContext) -> Attachment: ...
    def generate_attachment_id(self, mime_type: str, context: TContext) -> str: ...
```

## Message Lifecycle

### ThreadItemConverter
Converts between ChatKit and Agent SDK formats:

```python
from chatkit.agents import ThreadItemConverter

class MyThreadConverter(ThreadItemConverter):
    async def attachment_to_message_content(self, attachment: Attachment) -> ResponseInputTextParam:
        # Implementation
        pass
```

## Widget Components

### Text Component
```python
from chatkit.widgets import Text

Text(
    value="Hello, World!",
    size="md",
    color="primary",
    streaming=True
)
```

### Markdown Component
```python
from chatkit.widgets import Markdown

Markdown(
    value="# Hello\nThis is **markdown** content.",
    streaming=True
)
```

### Box Component (Container)
```python
from chatkit.widgets import Box, Text

Box(
    direction="col",
    gap=10,
    children=[
        Text(value="Item 1"),
        Text(value="Item 2")
    ]
)
```

### Form Component
```python
from chatkit.widgets import Form, Text, EditableProps, Button, ActionConfig

Form(
    direction="col",
    onSubmitAction=ActionConfig(
        type="update_todo",
        payload={"id": "123"}
    ),
    children=[
        Text(value="Title", color="secondary", size="sm"),
        Text(
            value="Current title",
            editable=EditableProps(name="title", required=True),
        ),
        Button(label="Save", submit=True)
    ]
)
```

## Action Handling

### Strongly Typed Actions
```python
from pydantic import BaseModel, TypeAdapter, Field
from typing import Literal, Annotated
from chatkit.actions import Action

class ExamplePayload(BaseModel):
    id: int

ExampleAction = Action[Literal["example"], ExamplePayload]
OtherAction = Action[Literal["other"], None]

AppAction = Annotated[
    ExampleAction | OtherAction,
    Field(discriminator="type"),
]

ActionAdapter: TypeAdapter[AppAction] = TypeAdapter(AppAction)

def parse_app_action(action: Action[str, Any]) -> AppAction:
    return ActionAdapter.validate_python(action)
```

## Streaming and Responses

### Streaming Results
```python
from chatkit.models import StreamingResult

if isinstance(result, StreamingResult):
    return StreamingResponse(result, media_type="text/event-stream")
else:
    return Response(content=result.json, media_type="application/json")
```

### Progress Updates
```python
from chatkit.types import ProgressUpdateEvent

@function_tool()
async def long_running_tool(ctx: RunContextWrapper[AgentContext]) -> str:
    await ctx.context.stream(
        ProgressUpdateEvent(text="Loading a user profile...")
    )
    await asyncio.sleep(1)
    return "Done"
```

## Client-Server Communication

### Context Management
```python
class MyChatKitServer(ChatKitServer):
    async def respond(
        self,
        thread: ThreadMetadata,
        input: UserMessageItem | None,
        context: Any,
    ) -> AsyncIterator[ThreadStreamEvent]:
        # Access context values like user ID
        user_id = context.get("userId", "anonymous")
        # Process with user-specific logic
```

### Imperative Action Sending
```typescript
// On the client side
await chatKit.sendAction({
  type: "example",
  payload: { id: 123 },
});
```
