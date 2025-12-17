from dataclasses import dataclass
from typing import List, Optional
from datetime import datetime
import uuid

@dataclass
class BookContentChunk:
    """
    A semantic unit of book content derived from Markdown files, suitable for embedding generation.
    """
    id: str
    content: str
    document_path: str
    section_type: str
    chapter_name: str
    section_heading: str
    language: str = "en"
    source_type: str = "book"
    module_name: Optional[str] = None
    indexing_version: Optional[str] = None
    embedding: Optional[List[float]] = None
    vector_id: Optional[str] = None
    heading_level: Optional[int] = None  # The heading level (1 for H1, 2 for H2, etc.)
    parent_headings: Optional[List[str]] = None  # Breadcrumbs of parent headings

    def __post_init__(self):
        """Validate the chunk after initialization."""
        if not self.content.strip():
            raise ValueError("Content cannot be empty")

        if not self.document_path:
            raise ValueError("Document path is required")

        if self.section_type not in ["intro", "hardware", "module"]:
            raise ValueError(f"Section type must be one of: intro, hardware, module. Got: {self.section_type}")

        if not self.chapter_name:
            raise ValueError("Chapter name is required")

        if not self.section_heading:
            raise ValueError("Section heading is required")

        if self.heading_level is not None and (self.heading_level < 1 or self.heading_level > 6):
            raise ValueError("Heading level must be between 1 and 6")

        if self.embedding is not None and len(self.embedding) != 1024:  # Default expected size
            raise ValueError(f"Embedding must have 1024 dimensions, got {len(self.embedding)}")