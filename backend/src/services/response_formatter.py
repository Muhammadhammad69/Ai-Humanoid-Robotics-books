from typing import Dict, Any, Optional
from ..models.agent import AgentResponse
from ..models.query import QueryResponse


class ResponseFormatterService:
    """
    Service for formatting agent responses to match the API response schema
    """

    def __init__(self):
        pass

    def format_agent_response_to_query_response(
        self,
        agent_response: AgentResponse,
        original_query: str,
        original_context: str,
        retrieved_chunks: Any,
        session_id: Optional[str] = None,
        query_id: Optional[str] = None
    ) -> QueryResponse:
        """
        Format the agent response to match the existing QueryResponse schema
        while including the agent-generated answer
        """
        # Calculate total processing time (we'll use agent processing time as part of the total)
        total_processing_time = agent_response.processing_time

        # Create the formatted response that extends the original QueryResponse
        formatted_response = QueryResponse(
            query=original_query,
            context=original_context,  # Original context from RAG pipeline
            agent_answer=agent_response.agent_answer,  # Agent-generated answer
            retrieved_chunks=retrieved_chunks,
            session_id=session_id,
            query_id=query_id,
            processing_time=total_processing_time
        )

        return formatted_response

    def format_error_response(
        self,
        query: str,
        error_message: str,
        session_id: Optional[str] = None,
        query_id: Optional[str] = None
    ) -> QueryResponse:
        """
        Format an error response that maintains compatibility with the API schema
        """
        return QueryResponse(
            query=query,
            context="",
            agent_answer=f"Error: {error_message}",
            retrieved_chunks=[],
            session_id=session_id,
            query_id=query_id,
            processing_time=0.0
        )


# Global instance for use in other modules
response_formatter_service = ResponseFormatterService()