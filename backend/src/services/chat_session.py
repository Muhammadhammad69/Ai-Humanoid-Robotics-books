import uuid
from datetime import datetime
from typing import Dict, Optional, List
from ..models.session import SessionState, Message
from ..config.settings import Settings


class ChatSessionService:
    """
    Service for managing chat sessions and message history.
    """

    def __init__(self):
        self.settings = Settings()
        self.sessions: Dict[str, SessionState] = {}
        # In a real implementation, this would connect to a persistent store like Redis or a database

    async def create_session(self, metadata: Optional[Dict] = None) -> SessionState:
        """
        Implement session creation functionality (task T038).
        """
        session_id = str(uuid.uuid4())
        created_at = datetime.now()

        session_state = SessionState(
            session_id=session_id,
            history=[],
            created_at=created_at,
            last_accessed=created_at,
            metadata=metadata or {}
        )

        self.sessions[session_id] = session_state
        return session_state

    async def get_session(self, session_id: str) -> Optional[SessionState]:
        """
        Implement session retrieval functionality (task T039).
        """
        session = self.sessions.get(session_id)
        if session:
            # Update last accessed time
            session.last_accessed = datetime.now()
        return session

    async def update_session(self, session_id: str, session_state: SessionState) -> bool:
        """
        Add session state management (task T040).
        """
        if session_id in self.sessions:
            self.sessions[session_id] = session_state
            return True
        return False

    async def delete_session(self, session_id: str) -> bool:
        """
        Delete a session.
        """
        if session_id in self.sessions:
            del self.sessions[session_id]
            return True
        return False

    async def add_message_to_session(self, session_id: str, message: Message) -> bool:
        """
        Add message history tracking (task T042).
        """
        session = await self.get_session(session_id)
        if session:
            # Check if we're at the history limit
            if len(session.history) >= self.settings.MAX_SESSION_HISTORY:
                # Remove oldest messages if we're at the limit
                excess = len(session.history) - self.settings.MAX_SESSION_HISTORY + 1
                session.history = session.history[excess:]

            # Add the new message
            session.history.append(message)

            # Update the session
            await self.update_session(session_id, session)
            return True
        return False

    async def get_session_history(self, session_id: str) -> List[Message]:
        """
        Get the message history for a session.
        """
        session = await self.get_session(session_id)
        if session:
            return session.history
        return []

    async def clear_session_history(self, session_id: str) -> bool:
        """
        Clear the message history for a session.
        """
        session = await self.get_session(session_id)
        if session:
            session.history = []
            await self.update_session(session_id, session)
            return True
        return False

    # ChatKit integration methods (tasks T045, T046, T047)
    async def integrate_with_chatkit(self):
        """
        Placeholder for ChatKit integration (tasks T045, T046, T047).
        In a real implementation, this would connect to ChatKit for session management.
        """
        pass

    async def create_chatkit_session(self, session_id: str, user_id: str = None):
        """
        Create a ChatKit session.
        """
        pass

    async def add_message_to_chatkit_session(self, session_id: str, message: Message):
        """
        Add a message to a ChatKit session.
        """
        pass