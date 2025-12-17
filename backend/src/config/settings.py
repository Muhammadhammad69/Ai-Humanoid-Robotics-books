import os
from typing import Optional
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class Settings:
    """Configuration settings loaded from environment variables."""

    # Qdrant Configuration
    QDRANT_URL: str = os.getenv("QDRANT_URL", "")
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    QDRANT_COLLECTION_NAME: str = os.getenv("QDRANT_COLLECTION_NAME", "book_embeddings_en")
    QDRANT_VECTOR_SIZE: int = int(os.getenv("QDRANT_VECTOR_SIZE", "1024"))
    QDRANT_DISTANCE: str = os.getenv("QDRANT_DISTANCE", "Cosine")

    # Cohere Configuration
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")
    COHERE_MODEL: str = os.getenv("COHERE_MODEL", "embed-english-v3.0")
    COHERE_OUTPUT_DIM: int = int(os.getenv("COHERE_OUTPUT_DIM", "1024"))
    COHERE_RATE_LIMIT_RPM: int = int(os.getenv("COHERE_RATE_LIMIT_RPM", "60"))
    COHERE_BATCH_SIZE: int = int(os.getenv("COHERE_BATCH_SIZE", "10"))

    # Gemini API Configuration
    GEMINI_API_KEY: str = os.getenv("GEMINI_API_KEY", "")
    GEMINI_BASE_URL: str = os.getenv("GEMINI_BASE_URL", "https://generativelanguage.googleapis.com/v1beta/openai/")
    GEMINI_MODEL: str = os.getenv("GEMINI_MODEL", "gemini-2.5-flash")

    # Ingestion Configuration
    DOCS_PATH: str = os.getenv("DOCS_PATH", "./docs")
    CHUNK_SIZE_TOKENS: int = int(os.getenv("CHUNK_SIZE_TOKENS", "512"))

    # RAG Pipeline Configuration
    TOP_K_RESULTS: int = int(os.getenv("TOP_K_RESULTS", "5"))
    RELEVANCE_THRESHOLD: float = float(os.getenv("RELEVANCE_THRESHOLD", "0.3"))
    TOKEN_BUDGET: int = int(os.getenv("TOKEN_BUDGET", "2000"))
    MAX_SESSION_HISTORY: int = int(os.getenv("MAX_SESSION_HISTORY", "50"))
    QUERY_TIMEOUT_SECONDS: int = int(os.getenv("QUERY_TIMEOUT_SECONDS", "30"))
    DUPLICATE_THRESHOLD: float = float(os.getenv("DUPLICATE_THRESHOLD", "0.95"))

    # Agent Configuration
    AGENT_TEMPERATURE: float = float(os.getenv("AGENT_TEMPERATURE", "0.7"))
    AGENT_MAX_TOKENS: int = int(os.getenv("AGENT_MAX_TOKENS", "1000"))
    AGENT_TIMEOUT: int = int(os.getenv("AGENT_TIMEOUT", "30"))
    AGENT_RETRY_ATTEMPTS: int = int(os.getenv("AGENT_RETRY_ATTEMPTS", "3"))

    @classmethod
    def validate(cls) -> None:
        """Validate that all required environment variables are set."""
        required_vars = [
            "QDRANT_URL",
            "QDRANT_API_KEY",
            "QDRANT_COLLECTION_NAME",
            "COHERE_API_KEY",
            "GEMINI_API_KEY"
        ]

        missing_vars = []
        for var in required_vars:
            value = getattr(cls, var)
            if not value:
                missing_vars.append(var)

        if missing_vars:
            raise ValueError(f"Missing required environment variables: {', '.join(missing_vars)}")

        # Validate vector size consistency
        if cls.QDRANT_VECTOR_SIZE != cls.COHERE_OUTPUT_DIM:
            raise ValueError(
                f"QDRANT_VECTOR_SIZE ({cls.QDRANT_VECTOR_SIZE}) must match "
                f"COHERE_OUTPUT_DIM ({cls.COHERE_OUTPUT_DIM})"
            )

        # Validate positive integer values
        if cls.QDRANT_VECTOR_SIZE <= 0:
            raise ValueError("QDRANT_VECTOR_SIZE must be a positive integer")

        if cls.COHERE_OUTPUT_DIM <= 0:
            raise ValueError("COHERE_OUTPUT_DIM must be a positive integer")

        if cls.COHERE_RATE_LIMIT_RPM <= 0:
            raise ValueError("COHERE_RATE_LIMIT_RPM must be a positive integer")

        if cls.COHERE_BATCH_SIZE <= 0:
            raise ValueError("COHERE_BATCH_SIZE must be a positive integer")

        if cls.CHUNK_SIZE_TOKENS <= 0:
            raise ValueError("CHUNK_SIZE_TOKENS must be a positive integer")

        if cls.AGENT_MAX_TOKENS <= 0:
            raise ValueError("AGENT_MAX_TOKENS must be a positive integer")

        if cls.AGENT_TIMEOUT <= 0:
            raise ValueError("AGENT_TIMEOUT must be a positive integer")

        if cls.AGENT_RETRY_ATTEMPTS <= 0:
            raise ValueError("AGENT_RETRY_ATTEMPTS must be a positive integer")


# Validate settings at import time
Settings.validate()