from fastapi import APIRouter, HTTPException
import uuid
from datetime import datetime
from ...models.session import SessionState, Message
from ...services.chat_session import ChatSessionService

router = APIRouter()

# Initialize the chat session service
chat_session_service = ChatSessionService()


@router.post("/session/create", response_model=SessionState)
async def create_session():
    """
    Create a new chat session with optional initial metadata.
    """
    try:
        session_state = await chat_session_service.create_session()
        return session_state
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to create session: {str(e)}")


@router.get("/session/{session_id}", response_model=SessionState)
async def get_session(session_id: str):
    """
    Retrieve session details and history.
    """
    try:
        session_state = await chat_session_service.get_session(session_id)
        if session_state is None:
            raise HTTPException(status_code=404, detail="Session not found")
        return session_state
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to retrieve session: {str(e)}")


@router.delete("/session/{session_id}")
async def delete_session(session_id: str):
    """
    Delete a chat session.
    """
    try:
        success = await chat_session_service.delete_session(session_id)
        if not success:
            raise HTTPException(status_code=404, detail="Session not found")
        return {"message": "Session deleted successfully"}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to delete session: {str(e)}")