from langchain.text_splitter import RecursiveCharacterTextSplitter
from typing import List, Dict, Any
import re


class MDXTextSplitter:
    """
    Splits MDX files intelligently while preserving semantic structure
    and stripping JSX/React components
    """

    def __init__(self, chunk_size: int = 1024, chunk_overlap: int = 256):
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap
        self.splitter = RecursiveCharacterTextSplitter(
            separators=[
                "\n## ",      # Markdown headings (highest priority)
                "\n### ",     # Sub-headings
                "\n\n",       # Paragraphs
                "\n",         # Lines
                " ",          # Words
                ""            # Characters
            ],
            chunk_size=chunk_size,
            chunk_overlap=chunk_overlap,
            length_function=len,
        )

    def extract_metadata(self, text: str, filename: str) -> Dict[str, Any]:
        """Extract module, chapter, section from frontmatter and filename"""
        metadata = {
            "filename": filename,
            "module": "",
            "chapter": "",
            "section": ""
        }

        # Extract from frontmatter (---id: module/chapter---)
        frontmatter_match = re.search(r'^---\s*\nid:\s*([^\n]+)', text, re.MULTILINE)
        if frontmatter_match:
            path = frontmatter_match.group(1).strip()
            parts = path.split('/')
            if len(parts) >= 2:
                metadata["module"] = parts[0]
                metadata["chapter"] = parts[1]

        # Extract title (first h1 heading)
        title_match = re.search(r'^#\s+(.+?)$', text, re.MULTILINE)
        if title_match:
            metadata["section"] = title_match.group(1).strip()

        return metadata

    def split_mdx_file(
        self,
        mdx_content: str,
        filename: str
    ) -> List[Dict[str, Any]]:
        """
        Split MDX file into chunks with metadata

        Returns:
            List of dicts with keys: 'text', 'module', 'chapter', 'section', 'chunk_id'
        """
        from .mdx_parser import strip_mdx_syntax

        # Extract metadata before stripping MDX
        metadata = self.extract_metadata(mdx_content, filename)

        # Strip MDX/JSX syntax
        clean_text = strip_mdx_syntax(mdx_content)

        # Split into chunks using RecursiveCharacterTextSplitter
        chunks = self.splitter.split_text(clean_text)

        # Create chunk records with metadata and IDs
        result = []
        for i, chunk in enumerate(chunks):
            if chunk.strip():  # Skip empty chunks
                result.append({
                    "text": chunk,
                    "module": metadata["module"],
                    "chapter": metadata["chapter"],
                    "section": metadata["section"],
                    "chunk_id": f"{filename}_{i}",
                    "chunk_index": i,
                    "total_chunks": len(chunks)
                })

        return result
