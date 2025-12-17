from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .routes import query, session
from .routes.health import router as health_router  # Added for task T053
from .middleware.session import session_middleware
from ..config.settings import Settings
import asyncio


def create_app() -> FastAPI:
    """
    Create and configure the FastAPI application (task T044).
    """
    app = FastAPI(
        title="Backend RAG Query Pipeline API",
        description="API for processing queries through the RAG pipeline",
        version="1.0.0",
    )

    # Add CORS middleware
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],  # In production, specify actual origins
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Add session middleware
    app.middleware("http")(session_middleware)

    # Include API routes
    app.include_router(query.router, prefix="", tags=["query"])
    app.include_router(session.router, prefix="", tags=["session"])
    app.include_router(health_router, prefix="", tags=["health"])  # Added for task T053

    return app


# Create the main application instance
app = create_app()


@app.on_event("startup")
async def startup_event():
    """
    Startup event handler.
    """
    print("Starting up RAG Query Pipeline API...")


@app.on_event("shutdown")
async def shutdown_event():
    """
    Shutdown event handler (task T050).
    """
    print("Shutting down RAG Query Pipeline API...")


# Additional API documentation can be added here (task T052)
@app.get("/")
async def root():
    """
    Root endpoint with API information.
    """
    return {
        "message": "RAG Query Pipeline API",
        "version": "1.0.0",
        "endpoints": {
            "query": "/query (POST)",
            "session_create": "/session/create (POST)",
            "session_get": "/session/{session_id} (GET)",
            "health": "/health (GET)"
        }
    }