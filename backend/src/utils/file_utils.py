import os
from pathlib import Path
from typing import List, Generator


def find_markdown_files(docs_path: str, exclude_dirs: List[str] = None) -> Generator[Path, None, None]:
    """
    Recursively find all Markdown files in the given directory, excluding specified directories.

    Args:
        docs_path: The root directory to search for Markdown files
        exclude_dirs: List of directory names to exclude from the search

    Yields:
        Path objects for each Markdown file found
    """
    if exclude_dirs is None:
        exclude_dirs = ["assets", "images", "diagrams", "code-samples"]

    docs_path = Path(docs_path)

    for file_path in docs_path.rglob("*.md"):
        # Check if the file's parent directory is in the exclude list
        if any(excluded_dir in str(file_path.relative_to(docs_path)) for excluded_dir in exclude_dirs):
            continue

        yield file_path


def read_file_content(file_path: Path) -> str:
    """
    Read the content of a file and return it as a string.

    Args:
        file_path: Path to the file to read

    Returns:
        Content of the file as a string
    """
    with open(file_path, 'r', encoding='utf-8') as file:
        return file.read()


def get_file_info(file_path: Path, docs_path: str) -> dict:
    """
    Extract information about a file based on its path.

    Args:
        file_path: Path to the file
        docs_path: Root docs directory

    Returns:
        Dictionary with file information
    """
    relative_path = file_path.relative_to(Path(docs_path))

    # Extract section type from the path
    section_type = "intro"  # Default
    if "hardware" in str(relative_path):
        section_type = "hardware"
    elif "modules" in str(relative_path):
        section_type = "module"

    # Extract module name if applicable
    module_name = None
    if section_type == "module":
        # Extract module name from path like modules/module-1/...
        parts = str(relative_path).split(os.sep)
        for part in parts:
            if part.startswith("module-"):
                module_name = part
                break

    # Extract chapter name (filename without extension)
    chapter_name = file_path.stem

    return {
        "document_path": str(relative_path),
        "section_type": section_type,
        "module_name": module_name,
        "chapter_name": chapter_name
    }