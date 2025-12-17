import uvicorn
from src.api.main import app
from src.config.settings import Settings


def main():
    """
    Update main.py to initialize the complete application (task T054).
    """
    print("Starting RAG Query Pipeline API...")

    # Validate settings before starting
    try:
        Settings.validate()
        print("Configuration validated successfully")
    except ValueError as e:
        print(f"Configuration error: {e}")
        return

    # Run the FastAPI application
    uvicorn.run(
        "src.api.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,  # Set to False in production
        log_level="info"
    )


if __name__ == "__main__":
    main()
