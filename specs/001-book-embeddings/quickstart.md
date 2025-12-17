# Quickstart Guide: Book Content Embedding System

## Prerequisites

- Python 3.11+
- uv package manager
- Access to Cohere API (with valid API key)
- Access to Qdrant Cloud (with valid URL and API key)
- Book content available in `/docs` directory as Markdown files

## Setup

### 1. Navigate to Backend Directory
```bash
cd backend
```

### 2. Initialize the Project with uv
```bash
uv init
```

### 3. Install Dependencies
```bash
uv add qdrant-client cohere python-dotenv PyYAML markdown pytest
uv add --dev black isort mypy
```

### 4. Verify Environment Configuration
Ensure all required environment variables are set in `backend/.env`:
- QDRANT_URL
- QDRANT_API_KEY
- QDRANT_COLLECTION_NAME
- QDRANT_VECTOR_SIZE
- QDRANT_DISTANCE
- COHERE_API_KEY
- COHERE_MODEL
- COHERE_OUTPUT_DIM
- COHERE_RATE_LIMIT_RPM
- COHERE_BATCH_SIZE
- DOCS_PATH
- CHUNK_SIZE_TOKENS

## Running the Indexer

### 1. Run the Full Indexing Process
```bash
cd backend
uv run src/cli/indexer_cli.py
```

### 2. Run with Specific Options
```bash
# Index specific document only
uv run src/cli/indexer_cli.py --document docs/intro.md

# Perform a dry run without storing vectors
uv run src/cli/indexer_cli.py --dry-run

# Re-index all content
uv run src/cli/indexer_cli.py --reindex
```

## Project Structure

```
backend/
├── src/
│   ├── models/           # Data models for chunks and embeddings
│   ├── services/         # Business logic modules
│   ├── cli/              # Command-line interface
│   ├── config/           # Configuration management
│   └── utils/            # Utility functions
├── tests/                # Test suite
├── .env                  # Environment configuration
└── pyproject.toml        # Project dependencies
```

## Development

### Running Tests
```bash
uv run pytest
```

### Formatting Code
```bash
uv run black src/
uv run isort src/
```

### Environment Validation
```bash
# Check that all required environment variables are set
uv run python -c "from src.config.settings import Settings; Settings.validate()"
```

## Monitoring and Maintenance

### Check Indexing Progress
- Monitor console output during indexing
- Check Qdrant Cloud dashboard for vector count
- Review log files for any errors

### Re-indexing
- Run with `--reindex` flag to rebuild the entire vector store
- Old vectors will be safely replaced with new ones
- Process maintains data integrity throughout

## Troubleshooting

### Common Issues
1. **API Rate Limits**: If you encounter rate limit errors, verify COHERE_RATE_LIMIT_RPM is set correctly
2. **Invalid Dimensions**: If embeddings have wrong dimensions, check COHERE_OUTPUT_DIM matches QDRANT_VECTOR_SIZE
3. **File Access**: Ensure DOCS_PATH points to the correct directory with readable Markdown files

### API Health Checks
```bash
# Verify Cohere API access
python -c "import cohere; co = cohere.Client('YOUR_API_KEY'); print(co.check_api_key())"

# Verify Qdrant connection
python -c "from qdrant_client import QdrantClient; client = QdrantClient(url='YOUR_URL', api_key='YOUR_KEY'); print(client.get_collections())"
```