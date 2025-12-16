#!/usr/bin/env python3
"""
Multi-Agent Coordinator using the OpenAI Agents SDK

This script demonstrates how to create specialized agents and implement handoffs
between them for multi-agent coordination.
"""

import os
import asyncio
from openai import AsyncOpenAI
from agents import Agent, Runner, OpenAIChatCompletionsModel



def create_specialized_agents():
    """
    Create specialized agents for different tasks or languages.

    Returns:
        dict: Dictionary of specialized agents
    """
    # Get the API key and base URL from environment
    api_key = os.getenv("OPENAI_API_KEY", "")
    base_url = os.getenv("OPENAI_BASE_URL", "")  # Default to OpenAI if not set

    # Create AsyncOpenAI client
    if base_url:
        client = AsyncOpenAI(base_url=base_url, api_key=api_key)
    else:
        client = AsyncOpenAI(api_key=api_key)

    # Create model instance using the client
    model_name = os.getenv("OPENAI_MODEL", "gpt-4o")
    model_instance = OpenAIChatCompletionsModel(model=model_name, openai_client=client)

    # Create specialized agents
    spanish_agent = Agent(
        name="Spanish Agent",
        instructions="You only speak Spanish. Be polite and helpful in Spanish.",
        model=model_instance
    )

    english_agent = Agent(
        name="English Agent",
        instructions="You only speak English. Be polite and helpful in English.",
        model=model_instance
    )

    french_agent = Agent(
        name="French Agent",
        instructions="You only speak French. Be polite and helpful in French.",
        model=model_instance
    )

    # Create triage agent that routes to specialists
    triage_agent = Agent(
        name="Triage Agent",
        instructions="Handoff to the appropriate agent based on the language of the request. "
                    "If the user speaks Spanish, handoff to the Spanish agent. "
                    "If the user speaks French, handoff to the French agent. "
                    "Otherwise, use the English agent.",
        model=model_instance,
        handoffs=[spanish_agent, english_agent, french_agent]
    )

    return {
        "spanish": spanish_agent,
        "english": english_agent,
        "french": french_agent,
        "triage": triage_agent
    }


def create_tool_agents():
    """
    Create agents with different specialized tools.

    Returns:
        dict: Dictionary of tool-enabled agents
    """
    from agents import function_tool

    @function_tool
    def get_weather(city: str) -> str:
        """Get current weather for a city."""
        # This is a mock implementation
        import random
        conditions = ["sunny", "cloudy", "rainy", "snowy"]
        temp = random.randint(15, 35)
        return f"The weather in {city} is {random.choice(conditions)} with a temperature of {temp}°C"

    @function_tool
    def calculate_math(expression: str) -> float:
        """Perform a mathematical calculation."""
        # This is a simple implementation - in practice, use a safe eval or proper parser
        try:
            # For safety, only allow basic mathematical operations
            allowed_chars = set('0123456789+-*/(). ')
            if not all(c in allowed_chars for c in expression):
                raise ValueError("Invalid characters in expression")
            return eval(expression)
        except:
            raise ValueError("Invalid mathematical expression")

    weather_agent = Agent(
        name="Weather Assistant",
        instructions="You are a weather assistant. Use the get_weather tool to provide weather information.",
        model=os.getenv("OPENAI_MODEL", "gpt-4o"),
        tools=[get_weather]
    )

    calculator_agent = Agent(
        name="Calculator Assistant",
        instructions="You are a calculator assistant. Use the calculate_math tool to perform mathematical operations.",
        model=os.getenv("OPENAI_MODEL", "gpt-4o"),
        tools=[calculate_math]
    )

    return {
        "weather": weather_agent,
        "calculator": calculator_agent
    }


async def run_agent_with_input(agent, user_input):
    """
    Run an agent with the given input.

    Args:
        agent (Agent): The agent to run
        user_input (str): User input to process

    Returns:
        str: Agent's response
    """
    result = await Runner.run(agent, user_input)
    return result.final_output


def main():
    print("Creating specialized agents...")

    # Create specialized agents
    specialized_agents = create_specialized_agents()
    print(f"Created {len(specialized_agents)} specialized agents: {list(specialized_agents.keys())}")

    # Create tool agents
    tool_agents = create_tool_agents()
    print(f"Created {len(tool_agents)} tool agents: {list(tool_agents.keys())}")

    print("\nAgents ready for multi-agent coordination.")
    print("Use run_agent_with_input(agent, user_input) to interact with any agent.")


if __name__ == "__main__":
    main()