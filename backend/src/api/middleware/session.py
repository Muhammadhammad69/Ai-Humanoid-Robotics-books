from fastapi import Request
from typing import Optional
import time
from ...services.chat_session import ChatSessionService


class SessionMiddleware:
    """
    Middleware for session handling and management.
    """

    def __init__(self):
        self.chat_session_service = ChatSessionService()

    async def __call__(self, request: Request, call_next):
        # Get session ID from header or create a new one
        session_id = request.headers.get("X-Session-ID")

        if not session_id:
            # Create a new session if none provided
            session_state = await self.chat_session_service.create_session()
            session_id = session_state.session_id
            request.state.session_id = session_id
        else:
            request.state.session_id = session_id
            # Verify session exists
            existing_session = await self.chat_session_service.get_session(session_id)
            if not existing_session:
                # Create new session if provided ID doesn't exist
                session_state = await self.chat_session_service.create_session()
                request.state.session_id = session_state.session_id

        # Add start time for performance tracking
        start_time = time.time()
        request.state.start_time = start_time

        # Process the request
        response = await call_next(request)

        # Add session ID to response headers
        response.headers["X-Session-ID"] = request.state.session_id

        return response


# FastAPI middleware function
async def session_middleware(request: Request, call_next):
    """
    FastAPI middleware for session handling (task T043).
    """
    middleware = SessionMiddleware()
    return await middleware(request, call_next)