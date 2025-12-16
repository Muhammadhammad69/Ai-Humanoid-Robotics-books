# OpenAI Agents SDK API Reference

## Core Components

### Agent
The `Agent` class represents an AI agent with specific instructions, model, and tools.

```python
from openai import AsyncOpenAI
from agents import Agent, ChatCompletionsModel


# Create client and model instance first
client = AsyncOpenAI(api_key="your-api-key")
model = OpenAIChatCompletionsModel(model="gpt-4o", openai_client=client)

agent = Agent(
    name="Agent Name",           # Name of the agent
    instructions="System instructions",  # Instructions for the agent
    model=model,                 # Model instance (not string)
    tools=[],                    # List of tools available to the agent
    handoffs=[]                  # List of agents to handoff to
)
```

### Runner
The `Runner` class executes agent runs.

```python
from agents import Runner

result = await Runner.run(
    agent,                      # Agent to run
    "User input",              # Input from user
    stream=False,              # Whether to stream the response
    session=None               # Session for conversation history
)
```

### Function Tool
The `function_tool` decorator creates tools from Python functions.

```python
from agents import function_tool

@function_tool
def my_tool(param: str) -> str:
    """Description of what the tool does."""
    # Implementation here
    return "result"
```

## Configuration Options

### Environment Variables
- `OPENAI_API_KEY` - API key for OpenAI
- `OPENAI_BASE_URL` - Base URL for OpenAI-compatible API (for other providers)
- `OPENAI_DEFAULT_MODEL` - Default model to use

### Custom Client Configuration
```python
from openai import AsyncOpenAI
from agents import Agent
from agents.models import OpenAIChatCompletionsModel

client = AsyncOpenAI(
    api_key="your-api-key",
    base_url="https://custom-provider.com/v1"
)

# Create model instance with the custom client
model = OpenAIChatCompletionsModel(model="your-model-name", openai_client=client)

# Use the model instance when creating an agent
agent = Agent(
    name="Custom Agent",
    instructions="You are a helpful assistant.",
    model=model
)
```

## Session Management

### Custom Session Implementation
```python
from agents.memory.session import SessionABC
from agents.items import TResponseInputItem
from typing import List

class MyCustomSession(SessionABC):
    def __init__(self, session_id: str):
        self.session_id = session_id

    async def get_items(self, limit: int | None = None) -> List[TResponseInputItem]:
        # Implementation
        pass

    async def add_items(self, items: List[TResponseInputItem]) -> None:
        # Implementation
        pass

    async def pop_item(self) -> TResponseInputItem | None:
        # Implementation
        pass

    async def clear_session(self) -> None:
        # Implementation
        pass
```

## Multi-Agent Coordination

### Handoffs
Agents can hand off to other agents based on conditions:

```python
specialist_agent = Agent(
    name="Specialist",
    instructions="Handle specialized tasks."
)

main_agent = Agent(
    name="Main",
    instructions="Handle general tasks, hand off to specialist when needed.",
    handoffs=[specialist_agent]
)
```

### Agent as Tool
Agents can be used as tools within other agents:

```python
translation_agent = Agent(
    name="Translator",
    instructions="Translate text between languages."
)

main_agent = Agent(
    name="Main",
    instructions="Handle requests, use translation when needed.",
    tools=[
        translation_agent.as_tool(
            tool_name="translate",
            tool_description="Translate text between languages"
        )
    ]
)
```

## Error Handling

### Custom Error Functions
```python
from agents import function_tool, RunContextWrapper
from typing import Any

def my_error_function(context: RunContextWrapper[Any], error: Exception) -> str:
    """Custom error handling function."""
    return "A custom error message"

@function_tool(failure_error_function=my_error_function)
def error_prone_function():
    # Function that might fail
    pass
```

## Async Operations

### Streaming Responses
```python
result = await Runner.run(agent, "input", stream=True)

async for chunk in result.stream():
    print(chunk)
```

### Retry Logic
```python
import asyncio

async def run_with_retry(agent, input_text, max_retries=3):
    for attempt in range(max_retries):
        try:
            result = await Runner.run(agent, input_text)
            return result
        except Exception as e:
            if attempt == max_retries - 1:
                raise e
            await asyncio.sleep(2 ** attempt)  # Exponential backoff
```

## Supported LLM Providers

### OpenAI
```python
import os
os.environ["OPENAI_API_KEY"] = "your-openai-api-key"

agent = Agent(
    name="OpenAI Agent",
    model="gpt-4o"
)
```

### Google Gemini (via OpenAI-compatible endpoint)
```python
import os
os.environ["OPENAI_BASE_URL"] = "https://generativelanguage.googleapis.com/v1beta/openai/"
os.environ["OPENAI_API_KEY"] = "your-gemini-api-key"

agent = Agent(
    name="Gemini Agent",
    model="gemini-1.5-pro"
)
```

### Other Providers (Ollama, Anthropic via proxy, etc.)
```python
import os
os.environ["OPENAI_BASE_URL"] = "https://your-provider.com/v1"
os.environ["OPENAI_API_KEY"] = "your-api-key"

agent = Agent(
    name="Custom Agent",
    model="your-model"
)
```
