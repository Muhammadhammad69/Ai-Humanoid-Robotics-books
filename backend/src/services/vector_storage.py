import logging
from typing import List
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from src.config.settings import Settings
from src.models.embedding import VectorEmbedding
from src.models.payload import MetadataPayload
from src.models.vector_point import QdrantVectorPoint


class VectorStorageService:
    """
    Service to store vectors in Qdrant Cloud.
    """

    def __init__(self):
        self.settings = Settings()
        self.client = QdrantClient(
            url=self.settings.QDRANT_URL,
            api_key=self.settings.QDRANT_API_KEY,
        )
        self.collection_name = self.settings.QDRANT_COLLECTION_NAME
        self.logger = logging.getLogger(__name__)
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        """Ensure the collection exists with proper configuration."""
        try:
            # Try to get collection info to see if it exists
            self.client.get_collection(self.collection_name)
        except Exception:
            # Collection doesn't exist, create it
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.settings.QDRANT_VECTOR_SIZE,
                    distance=Distance.COSINE,
                ),
            )

    def store_embedding(self, embedding: VectorEmbedding, payload: MetadataPayload):
        """
        Store a single embedding with its payload in Qdrant.

        Args:
            embedding: The VectorEmbedding to store
            payload: The MetadataPayload associated with the embedding
        """
        try:
            # Validate payload before storage
            self._validate_payload(payload)

            # Validate embedding dimensions
            if len(embedding.vector_data) != self.settings.QDRANT_VECTOR_SIZE:
                raise ValueError(f"Vector data has {len(embedding.vector_data)} dimensions, expected {self.settings.QDRANT_VECTOR_SIZE}")

            point = QdrantVectorPoint(
                point_id=embedding.vector_id,
                vector=embedding.vector_data,
                payload=payload,
                collection_name=self.collection_name
            )

            self.client.upsert(
                collection_name=self.collection_name,
                points=[
                    models.PointStruct(
                        id=point.point_id,
                        vector=point.vector,
                        payload=point.payload.to_dict()
                    )
                ]
            )

            self.logger.info(f"Successfully stored embedding with ID: {embedding.vector_id}")
        except Exception as e:
            self.logger.error(f"Failed to store embedding with ID {embedding.vector_id}: {str(e)}")
            raise e

    def _validate_payload(self, payload: MetadataPayload):
        """
        Validate that the payload contains all required metadata fields.

        Args:
            payload: The MetadataPayload to validate
        """
        if not payload.document_path:
            raise ValueError("document_path is required in payload")

        if payload.section_type not in ["intro", "hardware", "module"]:
            raise ValueError(f"section_type must be one of: intro, hardware, module. Got: {payload.section_type}")

        if not payload.chapter_name:
            raise ValueError("chapter_name is required in payload")

        if not payload.section_heading:
            raise ValueError("section_heading is required in payload")

    def store_embeddings(self, embeddings: List[VectorEmbedding], payloads: List[MetadataPayload]):
        """
        Store multiple embeddings with their payloads in Qdrant.

        Args:
            embeddings: List of VectorEmbedding objects to store
            payloads: List of MetadataPayload objects corresponding to the embeddings
        """
        if len(embeddings) != len(payloads):
            raise ValueError("Number of embeddings must match number of payloads")

        points = []
        for embedding, payload in zip(embeddings, payloads):
            # Validate each payload before creating the point
            self._validate_payload(payload)

            point = QdrantVectorPoint(
                point_id=embedding.vector_id,
                vector=embedding.vector_data,
                payload=payload,
                collection_name=self.collection_name
            )
            points.append(
                models.PointStruct(
                    id=point.point_id,
                    vector=point.vector,
                    payload=point.payload.to_dict()
                )
            )

        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

    def search(self, query_vector: List[float], limit: int = 10):
        """
        Search for similar vectors in the collection.

        Args:
            query_vector: The vector to search for similar items
            limit: Maximum number of results to return

        Returns:
            List of search results
        """
        # Check if the search method exists, otherwise try query_points
        if hasattr(self.client, 'search'):
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=limit
            )
        elif hasattr(self.client, 'query_points'):
            results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                limit=limit
            )
        else:
            raise AttributeError("Qdrant client does not have 'search' or 'query_points' method")

        return results

    def search_vectors(self, query_embedding: List[float], top_k: int = 10, filters: dict = None):
        """
        Search for similar vectors with filters and return formatted results.

        Args:
            query_embedding: The query embedding vector
            top_k: Number of top results to return
            filters: Dictionary of filters to apply

        Returns:
            List of RetrievedChunk objects
        """
        from ..models.query import RetrievedChunk
        from qdrant_client.http.models import Filter, FieldCondition, MatchValue

        # Build filter conditions if filters are provided
        search_filter = None
        if filters:
            conditions = []
            for key, value in filters.items():
                # Only add filters for fields that are indexed in Qdrant
                # Common indexed fields in our system: document_path, module, section_type, chapter_name
                # Skip language filter if it's not properly indexed
                if key in ['document_path', 'module', 'section_type', 'chapter_name']:
                    conditions.append(FieldCondition(
                        key=key,
                        match=MatchValue(value=value)
                    ))
                # Skip unsupported filters like 'language' that might not be indexed

            if conditions:
                search_filter = Filter(must=conditions)

        # Perform the search - use the same approach as the search method
        if hasattr(self.client, 'search'):
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                query_filter=search_filter
            )
            # For search method, results is a list of ScoredPoint objects
            points_list = results
        elif hasattr(self.client, 'query_points'):
            response = self.client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=top_k,
                query_filter=search_filter
            )
            # For query_points method, response is a QueryResponse object with points attribute
            # We need to access the points attribute to get the list of ScoredPoint objects
            from qdrant_client.http.models import QueryResponse
            if isinstance(response, QueryResponse):
                points_list = response.points
            else:
                # Fallback: if it's already a list (for backward compatibility)
                points_list = response
        else:
            raise AttributeError("Qdrant client does not have 'search' or 'query_points' method")

        # Format the results to match the RetrievedChunk model
        # Handle different result formats depending on the method used
        formatted_results = []
        for result in points_list:
            # Check if result is a ScoredPoint object (from search) or PointStruct (from query_points)
            # For ScoredPoint objects from query_points, payload is accessed as result.payload
            if hasattr(result, 'payload'):
                if isinstance(result.payload, dict):
                    # Direct dictionary access
                    raw_payload = result.payload
                else:
                    # If payload is a model object, convert to dict
                    raw_payload = getattr(result.payload, '__dict__', {})
                    if not raw_payload and hasattr(result.payload, 'dict'):
                        raw_payload = result.payload.dict()
                    elif not raw_payload:
                        raw_payload = {}
            else:
                # Fallback if no payload attribute
                raw_payload = getattr(result, '__dict__', {}).get('payload', {})
                if not raw_payload and hasattr(result, '_pb') and hasattr(result._pb, 'payload'):
                    raw_payload = result._pb.payload
                else:
                    raw_payload = {}

            # Extract content from the payload
            content = raw_payload.get('content', '')

            score = getattr(result, 'score', 0.0)

            # Use the properly extracted payload
            payload = raw_payload

            chunk_id = getattr(result, 'id', '')
            source_document = payload.get('document_path', '')

            chunk = RetrievedChunk(
                content=content,
                relevance_score=score,
                metadata=payload,
                chunk_id=chunk_id,
                source_document=source_document
            )
            formatted_results.append(chunk)

        return formatted_results

    def get_point(self, point_id: str):
        """
        Get a specific point by its ID.

        Args:
            point_id: The ID of the point to retrieve

        Returns:
            The point data
        """
        points = self.client.retrieve(
            collection_name=self.collection_name,
            ids=[point_id]
        )
        return points[0] if points else None

    def delete_point(self, point_id: str):
        """
        Delete a specific point by its ID.

        Args:
            point_id: The ID of the point to delete
        """
        self.client.delete(
            collection_name=self.collection_name,
            points_selector=models.PointIdsList(
                points=[point_id]
            )
        )

    def delete_points_by_payload_condition(self, payload_filter: dict):
        """
        Delete points that match a specific payload condition.

        Args:
            payload_filter: Dictionary containing payload field-value pairs to match for deletion
        """
        # Construct the selector based on payload filter
        conditions = []
        for key, value in payload_filter.items():
            conditions.append(models.FieldCondition(
                key=key,
                match=models.MatchValue(value=value)
            ))

        selector = models.Filter(must=conditions)

        self.client.delete(
            collection_name=self.collection_name,
            points_selector=selector
        )

    def get_points_by_document_path(self, document_path: str):
        """
        Get all points associated with a specific document path.

        Args:
            document_path: The document path to search for

        Returns:
            List of points matching the document path
        """
        filter_condition = models.Filter(
            must=[
                models.FieldCondition(
                    key="document_path",
                    match=models.MatchValue(value=document_path)
                )
            ]
        )

        points = self.client.scroll(
            collection_name=self.collection_name,
            scroll_filter=filter_condition,
            limit=10000  # Assuming we won't have more than 10k chunks per document
        )

        return points[0]  # Return the points list

    def get_points_by_indexing_version(self, indexing_version: str):
        """
        Get all points with a specific indexing version.

        Args:
            indexing_version: The indexing version to search for

        Returns:
            List of points matching the indexing version
        """
        filter_condition = models.Filter(
            must=[
                models.FieldCondition(
                    key="indexing_version",
                    match=models.MatchValue(value=indexing_version)
                )
            ]
        )

        points = self.client.scroll(
            collection_name=self.collection_name,
            scroll_filter=filter_condition,
            limit=10000
        )

        return points[0]  # Return the points list

    def count_total_points(self) -> int:
        """
        Get the total number of points in the collection.

        Returns:
            Total number of points in the collection
        """
        count_result = self.client.count(
            collection_name=self.collection_name
        )
        return count_result.count

    def clear_collection(self):
        """
        Delete all points in the collection.
        WARNING: This will remove ALL vectors from the collection.
        """
        # Delete all points by using a filter that matches everything
        self.client.delete(
            collection_name=self.collection_name,
            points_selector=models.Filter(must=[])  # Empty filter matches all points
        )