from dataclasses import dataclass
from typing import Optional

@dataclass
class MetadataPayload:
    """
    Information associated with each vector point that enables traceability back to source content.
    """
    document_path: str
    section_type: str
    chapter_name: str
    section_heading: str
    content: str  # The actual chunk text content
    language: str = "en"
    source_type: str = "book"
    module_name: Optional[str] = None
    indexing_version: Optional[str] = None

    def __post_init__(self):
        """Validate the payload after initialization."""
        if not self.document_path:
            raise ValueError("Document path is required")

        if self.section_type not in ["intro", "hardware", "module"]:
            raise ValueError(f"Section type must be one of: intro, hardware, module. Got: {self.section_type}")

        if not self.chapter_name:
            raise ValueError("Chapter name is required")

        if not self.section_heading:
            raise ValueError("Section heading is required")

        if not self.content:
            raise ValueError("Content is required")

        if self.language != "en":
            raise ValueError(f"Only 'en' language is currently supported, got: {self.language}")

        if self.source_type != "book":
            raise ValueError(f"Source type must be 'book', got: {self.source_type}")

    def to_dict(self) -> dict:
        """Convert the payload to a dictionary for Qdrant storage."""
        result = {
            "document_path": self.document_path,
            "section_type": self.section_type,
            "chapter_name": self.chapter_name,
            "section_heading": self.section_heading,
            "content": self.content,  # Include the actual chunk text
            "language": self.language,
            "source_type": self.source_type,
        }

        # Add optional fields only if they have values
        if self.module_name is not None:
            result["module_name"] = self.module_name
        if self.indexing_version is not None:
            result["indexing_version"] = self.indexing_version

        return result