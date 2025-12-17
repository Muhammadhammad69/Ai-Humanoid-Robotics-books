from dataclasses import dataclass
from typing import List, Dict, Any
from .payload import MetadataPayload

@dataclass
class QdrantVectorPoint:
    """
    Storage unit in Qdrant containing the vector embedding and associated metadata.
    """
    point_id: str
    vector: List[float]
    payload: MetadataPayload
    collection_name: str

    def __post_init__(self):
        """Validate the vector point after initialization."""
        if len(self.vector) != 1024:  # Expected size based on configuration
            raise ValueError(f"Vector must have 1024 dimensions, got {len(self.vector)}")

        if not self.point_id:
            raise ValueError("Point ID is required")

        if not self.collection_name:
            raise ValueError("Collection name is required")