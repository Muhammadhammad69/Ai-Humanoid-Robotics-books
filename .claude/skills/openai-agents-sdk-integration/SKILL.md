---
name: openai-agents-sdk-integration
description: This skill enables creating and managing OpenAI Agents SDK instances with support for multiple LLM providers including OpenAI, Gemini, and other LLMs. It provides tools for agent lifecycle management, tool integration, memory handling, and multi-agent coordination following the official OpenAI Agents SDK patterns.
---

# Openai Agents Sdk Integration

## Overview

This skill provides comprehensive tools for creating and managing AI agents using the OpenAI Agents SDK. It enables developers to create agents with different LLM providers (OpenAI, Gemini, or other LLMs) following the official custom agent integration patterns. The skill covers agent lifecycle management, tool integration, memory handling, and multi-agent coordination.

## Core Capabilities

### 1. Agent Creation with Multiple LLM Providers

Create agents using different LLM providers following the official custom model provider pattern:

```python
from openai import AsyncOpenAI
from agents import Agent, OpenAIChatCompletionsModel


# For OpenAI
client = AsyncOpenAI(api_key="your-api-key")
model = OpenAIChatCompletionsModel(model="gpt-4o", openai_client=client)
agent = Agent(
    name="OpenAI Agent",
    instructions="You are a helpful assistant.",
    model=model
)

# For Gemini (using compatible API)
client = AsyncOpenAI(
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
    api_key="your-gemini-api-key"
)
model = OpenAIChatCompletionsModel(model="gemini-2.5-flash", openai_client=client)
agent = Agent(
    name="Gemini Agent",
    instructions="You are a helpful assistant.",
    model=model
)

# For other LLMs (using compatible API)
client = AsyncOpenAI(
    base_url="https://your-llm-provider.com/v1",
    api_key="your-api-key"
)
model = OpenAIChatCompletionsModel(model="your-model-name", openai_client=client)
agent = Agent(
    name="Custom LLM Agent",
    instructions="You are a helpful assistant.",
    model=model
)
```

### 2. Agent Configuration and Initialization

Configure agents with proper instructions, models, and settings:

```python
from agents import Agent, Runner

# Create an agent with tools
@function_tool
def get_weather(city: str) -> str:
    """Get current weather for a city."""
    # Implementation here
    return f"The weather in {city} is sunny, 72°F"

agent = Agent(
    name="Weather Assistant",
    instructions="You are a helpful weather assistant. Use the get_weather tool to provide weather information.",
    model="gpt-4o",
    tools=[get_weather]
)
```

### 3. Tool and Action Integration

Integrate custom tools and actions with proper error handling:

```python
from agents import function_tool, RunContextWrapper
from typing import Any

@function_tool
def custom_api_call(param: str) -> str:
    """Perform a custom API call."""
    # Implementation here
    return f"Result for {param}"

# Tool with custom error handling
def my_custom_error_function(context: RunContextWrapper[Any], error: Exception) -> str:
    """A custom function to provide a user-friendly error message."""
    print(f"A tool call failed with the following error: {error}")
    return "An internal server error occurred. Please try again later."

@function_tool(failure_error_function=my_custom_error_function)
def error_prone_function(user_id: str) -> str:
    """A function that might fail."""
    if user_id == "valid_user":
        return "Success"
    else:
        raise ValueError(f"Invalid user ID: {user_id}")
```

### 4. Memory and State Management

Implement custom session memory for agents:

```python
from agents.memory.session import SessionABC
from agents.items import TResponseInputItem
from typing import List

class MyCustomSession(SessionABC):
    """Custom session implementation following the Session protocol."""

    def __init__(self, session_id: str):
        self.session_id = session_id
        # Your initialization here

    async def get_items(self, limit: int | None = None) -> List[TResponseInputItem]:
        """Retrieve conversation history for this session."""
        # Your implementation here
        pass

    async def add_items(self, items: List[TResponseInputItem]) -> None:
        """Store new items for this session."""
        # Your implementation here
        pass

    async def pop_item(self) -> TResponseInputItem | None:
        """Remove and return the most recent item from this session."""
        # Your implementation here
        pass

    async def clear_session(self) -> None:
        """Clear all items for this session."""
        # Your implementation here
        pass

# Use your custom session
agent = Agent(name="Assistant")
result = await Runner.run(
    agent,
    "Hello",
    session=MyCustomSession("my_session")
)
```

### 5. Multi-Agent Coordination

Create specialized agents and implement handoffs:

```python
# Create specialized agents
spanish_agent = Agent(
    name="Spanish Agent",
    instructions="You only speak Spanish."
)

english_agent = Agent(
    name="English Agent",
    instructions="You only speak English."
)

french_agent = Agent(
    name="French Agent",
    instructions="You only speak French."
)

# Create triage agent that routes to specialists
triage_agent = Agent(
    name="Triage Agent",
    instructions="Handoff to the appropriate agent based on the language of the request.",
    handoffs=[spanish_agent, english_agent, french_agent]
)
```

### 6. Async and Streaming Behavior

Handle async operations and streaming responses:

```python
import asyncio
from agents import Agent, Runner

async def run_agent_streaming():
    agent = Agent(
        name="Streaming Agent",
        instructions="You are a helpful assistant.",
        model="gpt-4o"
    )

    # Run with streaming
    result = await Runner.run(
        agent,
        "Hello, how are you?",
        stream=True
    )

    # Process streaming output
    async for chunk in result.stream():
        print(chunk)
```

### 7. Error Handling and Retries

Implement proper error handling and retry mechanisms:

```python
from agents import Agent, Runner
import asyncio

async def run_with_retry(agent, input_text, max_retries=3):
    for attempt in range(max_retries):
        try:
            result = await Runner.run(agent, input_text)
            return result
        except Exception as e:
            if attempt == max_retries - 1:
                raise e
            print(f"Attempt {attempt + 1} failed: {e}. Retrying...")
            await asyncio.sleep(2 ** attempt)  # Exponential backoff
```

## Security and API Key Management

### Setting API Keys Securely

```python
from agents import set_default_openai_key, set_default_openai_client
from openai import AsyncOpenAI

# Set API key programmatically
set_default_openai_key("sk-...", use_for_tracing=True)

# Or set custom client with different base URL for other LLM providers
client = AsyncOpenAI(
    api_key="your-api-key",
    base_url="https://custom-llm-provider.com/v1"
)
set_default_openai_client(client, use_for_tracing=True)
```

## Integration with FastAPI Backend

Create a FastAPI endpoint to manage agents:

```python
from fastapi import FastAPI, HTTPException
from agents import Agent, Runner
import asyncio
from pydantic import BaseModel

app = FastAPI()

class AgentRequest(BaseModel):
    message: str
    agent_type: str = "openai"  # openai, gemini, or custom

@app.post("/chat")
async def chat_with_agent(request: AgentRequest):
    try:
        # Initialize agent based on type
        if request.agent_type == "gemini":
            # Configure for Gemini
            import os
            os.environ["OPENAI_BASE_URL"] = "https://generativelanguage.googleapis.com/v1beta/openai/"
            agent = Agent(
                name="Gemini Agent",
                instructions="You are a helpful assistant.",
                model="gemini-2.5-flash"
            )
        elif request.agent_type == "custom":
            # Configure for custom LLM
            agent = Agent(
                name="Custom Agent",
                instructions="You are a helpful assistant.",
                model="your-model"
            )
        else:
            # Default to OpenAI
            agent = Agent(
                name="OpenAI Agent",
                instructions="You are a helpful assistant.",
                model="gpt-4o"
            )

        # Run the agent
        result = await Runner.run(agent, request.message)
        return {"response": result.final_output}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

## Usage Examples

### Basic Agent Creation

```python
from agents import Agent, Runner

# Create and run a simple agent
agent = Agent(
    name="Simple Assistant",
    instructions="You are a helpful assistant.",
    model="gpt-4o"
)

result = await Runner.run(agent, "Hello, how are you?")
print(result.final_output)
```

### Agent with Tools

```python
from agents import Agent, Runner, function_tool

@function_tool
def calculator(operation: str, a: float, b: float) -> float:
    """Perform a calculation."""
    if operation == "add":
        return a + b
    elif operation == "subtract":
        return a - b
    elif operation == "multiply":
        return a * b
    elif operation == "divide":
        if b == 0:
            raise ValueError("Cannot divide by zero")
        return a / b
    else:
        raise ValueError(f"Unknown operation: {operation}")

agent = Agent(
    name="Calculator Agent",
    instructions="You are a calculator assistant. Use the calculator tool to perform mathematical operations.",
    model="gpt-4o",
    tools=[calculator]
)

result = await Runner.run(agent, "What is 15.5 plus 24.3?")
print(result.final_output)
```

## Best Practices

1. Always set appropriate API keys securely
2. Implement proper error handling and retry logic
3. Use custom sessions for state management when needed
4. Follow the official OpenAI Agents SDK patterns for consistency
5. Test agents thoroughly before deployment
6. Use environment variables for API keys and configuration
7. Implement proper logging for debugging and monitoring
8. Consider rate limits when working with external APIs
