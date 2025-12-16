#!/bin/bash
# Script to set up a basic FastAPI chat orchestration project

set -e

PROJECT_NAME=${1:-"chat-orchestration-api"}
PYTHON_VERSION=${2:-"3.11"}

echo "Setting up FastAPI chat orchestration project: $PROJECT_NAME"

# Create project directory
mkdir -p $PROJECT_NAME
cd $PROJECT_NAME

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Create requirements.txt
cat > requirements.txt << 'EOF'
fastapi[standard]>=0.104.0
uvicorn[standard]>=0.24.0
pydantic>=2.5.0
pydantic-settings>=2.1.0
openai>=1.3.0
python-multipart>=0.0.6
websockets>=12.0
python-jose[cryptography]>=3.3.0
passlib[bcrypt]>=1.7.0
python-dotenv>=1.0.0
SQLAlchemy>=2.0.0
alembic>=1.13.0
EOF

# Install dependencies
pip install -r requirements.txt

# Create main application file
cat > main.py << 'EOF'
from contextlib import asynccontextmanager
from fastapi import FastAPI, Depends, HTTPException, status
from fastapi.security import HTTPBearer
from pydantic import BaseModel
from typing import Optional, List, Dict
import asyncio
import logging
import os
from datetime import datetime, timedelta
import jwt

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Security
security = HTTPBearer()

# Models
class ChatMessage(BaseModel):
    content: str
    role: str
    timestamp: Optional[str] = None

class ChatRequest(BaseModel):
    messages: List[ChatMessage]
    user_id: str
    thread_id: Optional[str] = None
    model: str = "gpt-4-turbo"

class ChatResponse(BaseModel):
    response: str
    thread_id: Optional[str] = None
    timestamp: str

# In-memory storage for demo (use database in production)
active_connections: Dict[str, any] = {}

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    logger.info("Starting up chat orchestration service")
    yield
    # Shutdown
    logger.info("Shutting down chat orchestration service")

app = FastAPI(
    title="Chat Orchestration API",
    description="API for managing chat sessions and messages",
    version="1.0.0",
    lifespan=lifespan
)

# Middleware
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Configure properly in production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Health check
@app.get("/health")
async def health_check():
    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat(),
        "service": "chat-orchestration-api"
    }

# Chat endpoint
@app.post("/api/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    try:
        # Process the chat request
        # In a real implementation, this would call the LLM service
        response_content = f"Echo: {request.messages[-1].content if request.messages else 'Hello'}"

        response = ChatResponse(
            response=response_content,
            thread_id=request.thread_id or f"thread_{len(active_connections) + 1}",
            timestamp=datetime.now().isoformat()
        )

        logger.info(f"Processed chat for user {request.user_id}")
        return response
    except Exception as e:
        logger.error(f"Chat processing error: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to process chat message")

# Session management (placeholder - integrate with ChatKit)
@app.post("/api/session")
async def create_session():
    # In a real implementation, this would create a ChatKit session
    return {"session_id": "session_123", "expires_at": (datetime.now() + timedelta(hours=1)).isoformat()}

# Authentication (simplified JWT example)
def create_access_token(data: dict, expires_delta: timedelta = None):
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, os.environ.get("SECRET_KEY", "secret"), algorithm="HS256")
    return encoded_jwt

@app.post("/token")
async def login():
    # In a real implementation, verify credentials here
    access_token_expires = timedelta(minutes=30)
    access_token = create_access_token(
        data={"sub": "user123"}, expires_delta=access_token_expires
    )
    return {"access_token": access_token, "token_type": "bearer"}

# Protected endpoint example
async def verify_token(credentials: HTTPBearer = Depends(security)):
    try:
        token = credentials.credentials
        payload = jwt.decode(token, os.environ.get("SECRET_KEY", "secret"), algorithms=["HS256"])
        user_id: str = payload.get("sub")
        if user_id is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials"
            )
        return user_id
    except jwt.JWTError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials"
        )

@app.post("/api/chat/protected", response_model=ChatResponse)
async def protected_chat_endpoint(
    request: ChatRequest,
    user_id: str = Depends(verify_token)
):
    request.user_id = user_id
    return await chat_endpoint(request)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True
    )
EOF

# Create .env file
cat > .env << 'EOF'
OPENAI_API_KEY=your_openai_api_key_here
SECRET_KEY=your_jwt_secret_key_here
DATABASE_URL=sqlite:///./chat.db
DEBUG=true
EOF

# Create README
cat > README.md << 'EOF'
# Chat Orchestration API

This is a FastAPI-based chat orchestration service that handles chat sessions, message processing, and integration with ChatKit.

## Setup

1. Create and activate virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Set up environment variables in `.env`:
   ```bash
   OPENAI_API_KEY=your_openai_api_key_here
   SECRET_KEY=your_jwt_secret_key_here
   ```

4. Run the application:
   ```bash
   uvicorn main:app --reload
   ```

## Endpoints

- `GET /health` - Health check
- `POST /api/chat` - Process chat messages
- `POST /api/session` - Create chat session
- `POST /token` - Get authentication token
- `POST /api/chat/protected` - Protected chat endpoint

## Features

- FastAPI with async support
- JWT authentication
- CORS enabled
- Chat message processing
- Session management
- Error handling
EOF

# Create pyproject.toml for project metadata
cat > pyproject.toml << 'EOF'
[tool.poetry]
name = "chat-orchestration-api"
version = "0.1.0"
description = "FastAPI-based chat orchestration service"
authors = ["Your Name <your.email@example.com>"]

[tool.poetry.dependencies]
python = "^3.11"
fastapi = {extras = ["standard"], version = "^0.104.0"}
uvicorn = {extras = ["standard"], version = "^0.24.0"}
pydantic = "^2.5.0"
pydantic-settings = "^2.1.0"
openai = "^1.3.0"
python-multipart = "^0.0.6"
websockets = "^12.0"
python-jose = {extras = ["cryptography"], version = "^3.3.0"}
passlib = {extras = ["bcrypt"], version = "^1.7.0"}
python-dotenv = "^1.0.0"
SQLAlchemy = "^2.0.0"
alembic = "^1.13.0"

[build-system]
requires = ["poetry-core"]
build-backend = "poetry.core.masonry.api"
EOF

echo "FastAPI chat orchestration project setup complete!"
echo "To run the application:"
echo "1. Update the .env file with your configuration"
echo "2. Activate the virtual environment: source venv/bin/activate"
echo "3. Run: uvicorn main:app --reload"
EOF