# chatbot_backend/rag_pipeline/text_chunker.py
from typing import List, Dict

def chunk_text(text: str, chunk_size: int = 500, chunk_overlap: int = 50) -> List[str]:
    """
    Splits a given text into smaller chunks with a specified overlap.
    Aims for fixed-size chunks, but adjusts for word boundaries.
    """
    words = text.split()
    chunks = []
    i = 0
    while i < len(words):
        chunk_words = words[i:i + chunk_size]
        chunks.append(" ".join(chunk_words))
        i += chunk_size - chunk_overlap
    return chunks

def chunk_documents(documents: List[Dict], chunk_size: int = 500, chunk_overlap: int = 50) -> List[Dict]:
    """
    Chunks a list of documents, where each document is a dictionary containing 'text_content'
    and 'front_matter'. Each chunk will retain the original document's front matter.
    """
    chunked_documents = []
    for doc in documents:
        text_content = doc['text_content']
        front_matter = doc['front_matter']
        file_path = doc['file_path']

        chunks = chunk_text(text_content, chunk_size, chunk_overlap)
        for i, chunk in enumerate(chunks):
            chunked_documents.append({
                "chunk_id": f"{front_matter.get('slug', file_path)}-chunk-{i}",
                "text_content": chunk,
                "front_matter": front_matter,
                "file_path": file_path,
                "chunk_number": i
            })
    return chunked_documents

if __name__ == "__main__":
    # Example usage:
    sample_text = """
    The Robot Operating System (ROS) is a flexible framework for writing robot software.
    It's a collection of tools, libraries, and conventions that aim to simplify the task
    of creating complex and robust robot behaviors across a wide variety of robotic platforms.
    ROS 2 is the latest iteration, re-architected to address the limitations of ROS 1,
    particularly concerning real-time performance, multi-robot systems, and embedded device support.
    At its core, ROS 2 replaces ROS 1's custom TCP/IP-based communication system with a
    Data Distribution Service (DDS) implementation. DDS is an open international standard for
    publish-subscribe communications in real-time systems. This change brings several benefits:
    Middleware Agnostic, Quality of Service (QoS), Improved Security, Multi-robot and Embedded Systems.
    """
    sample_document = {
        "text_content": sample_text,
        "front_matter": {"title": "Sample Doc", "slug": "/sample-doc"},
        "file_path": "path/to/sample.md"
    }

    chunked_docs = chunk_documents([sample_document], chunk_size=50, chunk_overlap=10)
    for i, chunk_doc in enumerate(chunked_docs):
        print(f"--- Chunk {i} ({chunk_doc['chunk_id']}) ---")
        print(f"Title: {chunk_doc['front_matter'].get('title')}")
        print(f"Content: {chunk_doc['text_content'][:200]}...")
        print("\n")
