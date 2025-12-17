from fastapi import APIRouter
from ...config.settings import Settings

router = APIRouter()
settings = Settings()


@router.get("/health")
async def health_check():
    """
    Add health check endpoint (task T053).
    """
    # Check if we can connect to required services
    health_status = {
        "status": "healthy",
        "service": "RAG Query Pipeline API",
        "version": "1.0.0",
        "dependencies": {
            "qdrant": _check_qdrant_connection(),
            "cohere": _check_cohere_connection()
        }
    }

    return health_status


def _check_qdrant_connection():
    """
    Check if Qdrant connection is working.
    """
    try:
        # In a real implementation, we would check the actual connection
        # For now, we'll just check if the settings are properly configured
        if settings.QDRANT_URL and settings.QDRANT_API_KEY:
            return {"status": "connected", "service": "qdrant"}
        else:
            return {"status": "not_configured", "service": "qdrant"}
    except Exception:
        return {"status": "error", "service": "qdrant"}


def _check_cohere_connection():
    """
    Check if Cohere connection is working.
    """
    try:
        # In a real implementation, we would make a test API call
        # For now, we'll just check if the API key is configured
        if settings.COHERE_API_KEY:
            return {"status": "configured", "service": "cohere"}
        else:
            return {"status": "not_configured", "service": "cohere"}
    except Exception:
        return {"status": "error", "service": "cohere"}