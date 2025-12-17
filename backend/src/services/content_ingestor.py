import uuid
from pathlib import Path
from typing import List, Generator
from src.utils.file_utils import find_markdown_files, read_file_content, get_file_info
from src.models.chunk import BookContentChunk


class ContentIngestor:
    """
    Service to scan and read Markdown files from the /docs directory.
    """

    def __init__(self, docs_path: str = "./docs"):
        self.docs_path = docs_path
        self.exclude_dirs = ["assets", "images", "diagrams", "code-samples"]

    def scan_documents(self) -> Generator[Path, None, None]:
        """
        Recursively scan the docs directory for Markdown files, excluding specified directories.

        Yields:
            Path objects for each Markdown file found
        """
        yield from find_markdown_files(self.docs_path, self.exclude_dirs)

    def read_document(self, file_path: Path) -> str:
        """
        Read the content of a Markdown file.

        Args:
            file_path: Path to the Markdown file

        Returns:
            Content of the file as a string
        """
        return read_file_content(file_path)

    def get_document_info(self, file_path: Path) -> dict:
        """
        Extract information about a document based on its path.

        Args:
            file_path: Path to the document

        Returns:
            Dictionary with document information
        """
        return get_file_info(file_path, self.docs_path)

    def get_all_documents(self) -> List[dict]:
        """
        Get all document information from the docs directory.

        Returns:
            List of dictionaries with document information
        """
        documents = []
        for file_path in self.scan_documents():
            doc_info = self.get_document_info(file_path)
            doc_info["content"] = self.read_document(file_path)
            documents.append(doc_info)
        return documents

    def process_documents(self, use_semantic_chunking: bool = True) -> Generator[BookContentChunk, None, None]:
        """
        Process all documents and yield BookContentChunk objects.

        Args:
            use_semantic_chunking: Whether to use semantic chunking (headings/paragraphs) or simple chunking

        Yields:
            BookContentChunk objects for each document
        """
        from src.services.chunker import BasicChunker

        for file_path in self.scan_documents():
            doc_info = self.get_document_info(file_path)
            content = self.read_document(file_path)

            if use_semantic_chunking:
                # Use enhanced semantic chunking
                chunker = BasicChunker()
                chunks = chunker.chunk_content_preserving_semantics(content, doc_info)

                for chunk in chunks:
                    yield chunk
            else:
                # Create a single chunk per document (original behavior)
                chunk = BookContentChunk(
                    id=str(uuid.uuid4()),
                    content=content,
                    document_path=doc_info["document_path"],
                    section_type=doc_info["section_type"],
                    module_name=doc_info.get("module_name"),
                    chapter_name=doc_info["chapter_name"],
                    section_heading="Full Document",  # Placeholder for now
                    language="en",
                    source_type="book"
                )
                yield chunk