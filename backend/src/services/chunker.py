import uuid
from typing import List
from src.models.chunk import BookContentChunk
from src.utils.markdown_parser import extract_headings, split_by_headings, split_by_paragraphs, clean_markdown_content


class BasicChunker:
    """
    Service to split content into chunks with semantic awareness.
    """

    def __init__(self, max_chunk_size: int = 512):  # Token-based chunking (approximated with characters)
        self.max_chunk_size = max_chunk_size

    def chunk_content(self, content: str, document_info: dict) -> List[BookContentChunk]:
        """
        Split content into chunks based on semantic boundaries (headings and paragraphs).

        Args:
            content: The content to chunk
            document_info: Information about the document (path, type, etc.)

        Returns:
            List of BookContentChunk objects
        """
        chunks = []

        # Extract headings from the content
        headings = extract_headings(content)

        if headings:
            # Split content by headings first
            heading_content_pairs = split_by_headings(content, headings)

            chunk_num = 0
            for heading_text, content_after_heading in heading_content_pairs:
                if content_after_heading.strip():
                    # Further split the content after each heading by paragraphs if it's too large
                    sub_chunks = split_by_paragraphs(content_after_heading, self.max_chunk_size)

                    for sub_chunk_content in sub_chunks:
                        if sub_chunk_content.strip():
                            chunk = BookContentChunk(
                                id=str(uuid.uuid4()),
                                content=sub_chunk_content,
                                document_path=document_info["document_path"],
                                section_type=document_info["section_type"],
                                module_name=document_info.get("module_name"),
                                chapter_name=document_info["chapter_name"],
                                section_heading=heading_text if heading_text else "Untitled",
                                language="en",
                                source_type="book"
                            )
                            chunks.append(chunk)
                            chunk_num += 1
        else:
            # If no headings found, split by paragraphs
            sub_chunks = split_by_paragraphs(content, self.max_chunk_size)
            chunk_num = 0

            for sub_chunk_content in sub_chunks:
                if sub_chunk_content.strip():
                    chunk = BookContentChunk(
                        id=str(uuid.uuid4()),
                        content=sub_chunk_content,
                        document_path=document_info["document_path"],
                        section_type=document_info["section_type"],
                        module_name=document_info.get("module_name"),
                        chapter_name=document_info["chapter_name"],
                        section_heading=document_info.get("section_heading", "Untitled"),
                        language="en",
                        source_type="book"
                    )
                    chunks.append(chunk)
                    chunk_num += 1

        return chunks

    def chunk_content_preserving_semantics(self, content: str, document_info: dict) -> List[BookContentChunk]:
        """
        Enhanced chunking that preserves document structure and semantic meaning.

        Args:
            content: The content to chunk
            document_info: Information about the document (path, type, etc.)

        Returns:
            List of BookContentChunk objects with preserved semantic meaning
        """
        chunks = []

        # Extract headings to understand document structure
        headings = extract_headings(content)

        if headings:
            # Process content in sections based on headings
            chunk_num = 0

            # Create a mapping from heading text to heading object for quick lookup
            heading_map = {h.text: h for h in headings}

            # Split content by headings
            heading_content_pairs = split_by_headings(content, headings)

            for heading_text, content_after_heading in heading_content_pairs:
                if content_after_heading.strip():
                    # Split large content blocks by paragraphs while preserving heading context
                    sub_chunks = split_by_paragraphs(content_after_heading, self.max_chunk_size)

                    for i, sub_chunk_content in enumerate(sub_chunks):
                        if sub_chunk_content.strip():
                            # Use the heading as the section heading, or create a more specific one if needed
                            section_heading = heading_text if heading_text else document_info.get("section_heading", "Untitled")

                            # If we have multiple sub-chunks from the same heading, indicate this
                            if len(sub_chunks) > 1:
                                section_heading = f"{heading_text} (Part {i+1})" if heading_text else f"Untitled (Part {i+1})"

                            # Get heading level if available
                            heading_level = None
                            if heading_text in heading_map:
                                heading_obj = heading_map[heading_text]
                                heading_level = heading_obj.level

                            chunk = BookContentChunk(
                                id=str(uuid.uuid4()),
                                content=sub_chunk_content,
                                document_path=document_info["document_path"],
                                section_type=document_info["section_type"],
                                module_name=document_info.get("module_name"),
                                chapter_name=document_info["chapter_name"],
                                section_heading=section_heading,
                                language="en",
                                source_type="book",
                                heading_level=heading_level
                            )
                            chunks.append(chunk)
                            chunk_num += 1
        else:
            # If no headings, just split by paragraphs
            sub_chunks = split_by_paragraphs(content, self.max_chunk_size)
            chunk_num = 0

            for sub_chunk_content in sub_chunks:
                if sub_chunk_content.strip():
                    chunk = BookContentChunk(
                        id=str(uuid.uuid4()),
                        content=sub_chunk_content,
                        document_path=document_info["document_path"],
                        section_type=document_info["section_type"],
                        module_name=document_info.get("module_name"),
                        chapter_name=document_info["chapter_name"],
                        section_heading=document_info.get("section_heading", "Untitled"),
                        language="en",
                        source_type="book"
                    )
                    chunks.append(chunk)
                    chunk_num += 1

        return chunks