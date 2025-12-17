import asyncio
import time
from typing import Optional, Dict, Any, List
from openai import AsyncOpenAI
from pydantic import BaseModel

from ..models.agent import AgentRequest, AgentResponse, AgentConfig
from ..config.settings import Settings


class AgentService:
    """
    Service for managing AI agent interactions using OpenAI-compatible API with Gemini LLM
    Note: Using chat completions API as the OpenAI Agents SDK may not be fully compatible with Gemini
    """

    def __init__(self):
        self.settings = Settings()
        self.client = AsyncOpenAI(
            api_key=self.settings.GEMINI_API_KEY,
            base_url=self.settings.GEMINI_BASE_URL
        )

    async def create_agent(self, agent_config: Optional[AgentConfig] = None) -> Dict[str, Any]:
        """
        Create an agent configuration for Gemini API usage
        """
        if agent_config is None:
            agent_config = AgentConfig(
                model_name=self.settings.GEMINI_MODEL,
                temperature=self.settings.AGENT_TEMPERATURE,
                max_tokens=self.settings.AGENT_MAX_TOKENS,
                timeout=self.settings.AGENT_TIMEOUT
            )

        return {
            "model": agent_config.model_name,
            "temperature": agent_config.temperature,
            "max_tokens": agent_config.max_tokens,
            "timeout": agent_config.timeout,
            "created_at": time.time()
        }

    async def generate_answer(self, agent_request: AgentRequest) -> AgentResponse:
        """
        Generate an answer using the AI agent with provided context
        """
        start_time = time.time()

        try:
            # Prepare context for LLM consumption
            prepared_context = await self.prepare_context_for_llm(agent_request.context)

            # Format the prompt for the LLM
            prompt = self._format_prompt(agent_request.query, prepared_context)

            # Get configuration
            config = agent_request.agent_config or AgentConfig(
                model_name=self.settings.GEMINI_MODEL,
                temperature=self.settings.AGENT_TEMPERATURE,
                max_tokens=self.settings.AGENT_MAX_TOKENS,
                timeout=self.settings.AGENT_TIMEOUT
            )

            # Call the Gemini API using the chat completions endpoint
            response = await self.client.chat.completions.create(
                model=config.model_name,
                messages=[
                    {"role": "system", "content": "You are a helpful AI assistant that answers questions based on provided context. Use only the information in the context to answer the user's query. If the context doesn't contain enough information to answer the query, say so."},
                    {"role": "user", "content": prompt}
                ],
                temperature=config.temperature,
                max_tokens=config.max_tokens,
                timeout=config.timeout
            )

            # Extract the answer from the response
            agent_answer = response.choices[0].message.content.strip()

            # Calculate processing time
            processing_time = time.time() - start_time

            # Create and return the agent response
            return AgentResponse(
                agent_answer=agent_answer,
                context_used=prepared_context,
                processing_time=processing_time,
                model_used=config.model_name,
                confidence_score=None  # Gemini doesn't provide confidence scores directly
            )

        except Exception as e:
            processing_time = time.time() - start_time
            raise Exception(f"Agent generation failed: {str(e)}") from e

    def _format_prompt(self, query: str, context: str) -> str:
        """
        Format the prompt for the LLM with query and context
        """
        return f"""
        Context: {context}

        Question: {query}

        Please provide a clear and concise answer based only on the information in the context above. If the context does not contain enough information to answer the question, please state that explicitly.
        """

    async def prepare_context_for_llm(self, context: str, max_length: int = 25000) -> str:
        """
        Prepare and format context data for LLM consumption with safety checks
        """
        # Truncate context if it exceeds maximum length
        if len(context) > max_length:
            context = context[:max_length] + "\n\n[Context truncated due to length limitations]"

        # Apply safety filtering to remove sensitive information
        from .context_formatter import context_formatter_service
        filtered_context = context_formatter_service.filter_sensitive_content(context)

        return filtered_context

    async def validate_response(self, response: str) -> bool:
        """
        Validate that the agent response is non-empty and coherent
        """
        if not response or len(response.strip()) == 0:
            return False

        # Additional validation could be added here
        # For example, checking for coherence, relevance, etc.

        return True


# Global instance for use in other modules
agent_service = AgentService()