# Agent Implementation Template

This template provides a starting point for implementing custom agents using the OpenAI Agents SDK.

## Basic Agent Template

```python
from openai import AsyncOpenAI
from agents import Agent, Runner, OpenAIChatCompletionsModel

import os

# Set your API key
api_key = os.getenv("OPENAI_API_KEY", "your-api-key-here")

# Create your client and model
client = AsyncOpenAI(api_key=api_key)
model = OpenAIChatCompletionsModel(model="gpt-4o", openai_client=client)  # or your preferred model

# Create your agent
agent = Agent(
    name="Your Agent Name",
    instructions="Your agent instructions here...",
    model=model,      # Model instance (not string)
    tools=[],         # Add your tools here
)

# Run the agent
async def run_agent():
    result = await Runner.run(agent, "Your input here")
    return result.final_output

# Example usage
# response = asyncio.run(run_agent())
# print(response)
```

## Agent with Tools Template

```python
from agents import Agent, Runner, function_tool
import os

# Define your custom tools
@function_tool
def your_custom_tool(param: str) -> str:
    """Description of what your tool does."""
    # Your tool implementation here
    return "result"

# Set your API key
os.environ["OPENAI_API_KEY"] = "your-api-key-here"

# Create your agent with tools
agent = Agent(
    name="Your Tool Agent",
    instructions="Your agent instructions here...",
    model="gpt-4o",
    tools=[your_custom_tool],
)

# Run the agent
async def run_agent_with_tools():
    result = await Runner.run(agent, "Ask the agent to use your tool")
    return result.final_output
```

## Multi-Agent Template

```python
from agents import Agent
import os

# Set your API key
os.environ["OPENAI_API_KEY"] = "your-api-key-here"

# Create specialized agents
specialist_agent = Agent(
    name="Specialist Agent",
    instructions="Handle specialized tasks...",
    model="gpt-4o",
)

# Create main agent that can hand off to specialist
main_agent = Agent(
    name="Main Agent",
    instructions="Handle general tasks, hand off to specialist when needed...",
    model="gpt-4o",
    handoffs=[specialist_agent],
)
```