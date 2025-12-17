import re
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class Heading:
    """Represents a markdown heading."""
    level: int  # 1 for H1, 2 for H2, etc.
    text: str
    start_pos: int  # Position in the text where the heading starts
    end_pos: int  # Position in the text where the heading ends


def extract_headings(content: str) -> List[Heading]:
    """
    Extract all headings from markdown content.

    Args:
        content: The markdown content to parse

    Returns:
        List of Heading objects
    """
    headings = []
    lines = content.split('\n')

    for i, line in enumerate(lines):
        # Match markdown headings: # Heading, ## Heading, etc.
        match = re.match(r'^(#{1,6})\s+(.+)$', line.strip())
        if match:
            level = len(match.group(1))
            text = match.group(2).strip()
            # Calculate position in the original content
            start_pos = sum(len(l) + 1 for l in lines[:i])  # +1 for newline
            end_pos = start_pos + len(line)
            headings.append(Heading(level, text, start_pos, end_pos))

    return headings


def split_by_headings(content: str, headings: List[Heading]) -> List[Tuple[str, str]]:
    """
    Split content by headings, returning tuples of (heading_text, content_after_heading).

    Args:
        content: The markdown content to split
        headings: List of Heading objects

    Returns:
        List of tuples containing (heading_text, content_after_heading)
    """
    if not headings:
        return [("", content)]

    chunks = []
    start = 0

    for heading in headings:
        # Add content before this heading (if any)
        if start < heading.start_pos:
            pre_content = content[start:heading.start_pos].strip()
            if pre_content:
                chunks.append(("", pre_content))

        # Find the next heading to determine the end position
        next_start = len(content)
        for next_heading in headings:
            if next_heading.start_pos > heading.start_pos:
                next_start = next_heading.start_pos
                break

        # Extract content after the current heading
        content_start = heading.end_pos + 1  # Skip the newline after heading
        content_end = next_start
        heading_content = content[content_start:content_end].strip()

        chunks.append((heading.text, heading_content))
        start = next_start

    return chunks


def split_by_paragraphs(content: str, max_chunk_size: int = 512) -> List[str]:
    """
    Split content into paragraphs, respecting the maximum chunk size.

    Args:
        content: The content to split
        max_chunk_size: Maximum size of each chunk (in characters)

    Returns:
        List of content chunks
    """
    paragraphs = content.split('\n\n')
    chunks = []
    current_chunk = ""

    for paragraph in paragraphs:
        # If adding this paragraph would exceed the max size, start a new chunk
        if len(current_chunk) + len(paragraph) > max_chunk_size and current_chunk:
            chunks.append(current_chunk.strip())
            current_chunk = paragraph
        else:
            if current_chunk:
                current_chunk += "\n\n" + paragraph
            else:
                current_chunk = paragraph

    # Add the last chunk if it's not empty
    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    return chunks


def clean_markdown_content(content: str) -> str:
    """
    Clean markdown content by removing unnecessary elements while preserving text.

    Args:
        content: Raw markdown content

    Returns:
        Cleaned content with markdown syntax removed
    """
    # Remove code blocks
    content = re.sub(r'```.*?```', '', content, flags=re.DOTALL)
    # Remove inline code
    content = re.sub(r'`(.*?)`', r'\1', content)
    # Remove bold and italic markers
    content = re.sub(r'\*\*(.*?)\*\*', r'\1', content)
    content = re.sub(r'\*(.*?)\*', r'\1', content)
    content = re.sub(r'__(.*?)__', r'\1', content)
    content = re.sub(r'_(.*?)_', r'\1', content)
    # Remove image syntax but keep alt text
    content = re.sub(r'!\[([^\]]*)\]\([^)]*\)', r'\1', content)
    # Remove link syntax but keep link text
    content = re.sub(r'\[([^\]]+)\]\([^)]*\)', r'\1', content)

    return content.strip()