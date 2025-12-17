import pytest
import os
from unittest.mock import patch
from src.config.settings import Settings


@pytest.fixture(scope="session", autouse=True)
def test_settings():
    """Override settings for testing environment."""
    # Set up test-specific environment variables
    with patch.dict(os.environ, {
        "QDRANT_URL": "https://test-qdrant-url.com",
        "QDRANT_API_KEY": "test-api-key",
        "QDRANT_COLLECTION_NAME": "test_collection",
        "QDRANT_VECTOR_SIZE": "1024",
        "QDRANT_DISTANCE": "Cosine",
        "COHERE_API_KEY": "test-cohere-key",
        "COHERE_MODEL": "embed-english-v3.0",
        "COHERE_OUTPUT_DIM": "1024",
        "COHERE_RATE_LIMIT_RPM": "60",
        "COHERE_BATCH_SIZE": "10",
        "DOCS_PATH": "./tests/fixtures",
        "CHUNK_SIZE_TOKENS": "512"
    }):
        # Reload settings to pick up the test environment variables
        import importlib
        import src.config.settings
        importlib.reload(src.config.settings)
        yield src.config.settings.Settings


@pytest.fixture
def sample_markdown_content():
    """Sample markdown content for testing."""
    return """# Introduction

This is the introduction section.

## Getting Started

Here's how to get started with the system.

### Prerequisites

- Python 3.11+
- uv package manager

## Main Concepts

These are the main concepts you need to understand.

Some additional content that doesn't have a heading.

## Advanced Usage

For advanced users, here are some tips.
"""


@pytest.fixture
def sample_chunk_data():
    """Sample chunk data for testing."""
    return {
        "id": "test-chunk-1",
        "content": "This is a sample chunk of content.",
        "document_path": "docs/intro.md",
        "section_type": "intro",
        "chapter_name": "intro",
        "section_heading": "Introduction"
    }