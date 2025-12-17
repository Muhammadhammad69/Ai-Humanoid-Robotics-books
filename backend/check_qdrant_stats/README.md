# Qdrant Stats Checker

This script checks the collection stats in Qdrant to verify that embeddings were successfully stored.

## Usage

```bash
python check_qdrant_stats.py
```

The script will:
- Connect to your Qdrant instance using the configured environment variables
- Check the total number of vectors stored in the collection
- Display additional collection information
- Verify that embeddings have been successfully stored

## Requirements

Make sure the following environment variables are set:
- `QDRANT_URL`: URL for Qdrant Cloud instance
- `QDRANT_API_KEY`: API key for Qdrant Cloud
- `QDRANT_COLLECTION_NAME`: Name of the collection to store embeddings (defaults to "book_embeddings_en")
- `QDRANT_VECTOR_SIZE`: Size of the embedding vectors (defaults to 1024)